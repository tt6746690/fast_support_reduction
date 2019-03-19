#include <Eigen/Core>

#include <igl/readMESH.h>
#include <igl/readOBJ.h>
#include <igl/readDMAT.h>
#include <igl/readTGF.h>
#include <igl/writeMESH.h>
#include <igl/writeOBJ.h>
#include <igl/slice.h>
#include <igl/jet.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/normalize_row_sums.h>
#include <igl/deform_skeleton.h>
#include <igl/copyleft/cgal/convex_hull.h>
#include <igl/centroid.h>

#include "reduce_support.h"
#include "overhang_energy.h"
#include "minitrace.h"
#include "support_polygon.h"

#include <cstdio>
#include <iostream>
#include <string>
#include <algorithm>
#include <numeric>
#include <vector>
#include <cmath>
#include <unistd.h>

using namespace std;
using namespace Eigen;


const auto rad2deg = [](double radian) {
    return (radian / 180) * M_PI; 
};


string filename, data_dir;
bool is3d, show_help;
int n_fixed_bones, pso_iters, pso_population;
double c_arap, c_overhang, c_intersect, rotation_angle;
Eigen::RowVector3d dp;  // printing direction

MatrixXd V, U, W, C, T;
MatrixXi Tet, F, BE;

int selected = 0;
const RowVector3d red(1., 0., 0.);
const RowVector3d green(0., 1., 0.);
const RowVector3d blue(0., 1., 0.);
const RowVector3d sea_green(70./255.,252./255.,167./255.);

int main(int argc, char*argv[]) 
{
    show_help      = false;
    data_dir       = "../data/";
    filename       = "woody";
    n_fixed_bones  = 0;
    pso_iters      = 1;
    pso_population = 1;
    dp             = Eigen::RowVector3d(0., 1., 0.);
    c_arap         = 1;
    c_overhang     = 1;
    c_intersect    = 1;
    rotation_angle = rad2deg(30);

    int c;
    while ((c = getopt (argc, argv, "d:f:b:i:p:r:a:c:e:")) != -1) {
        switch (c) {
            case 'd':
                data_dir = std::string(optarg);
                break;
            case 'f':
                filename = std::string(optarg);
                break;
            case 'b':
                n_fixed_bones = std::stoi(std::string(optarg));
                break;
            case 'i':
                pso_iters = std::stoi(std::string(optarg));
                break;
            case 'p':
                pso_population = std::stoi(std::string(optarg));
                break;
            case 'r':
                rotation_angle = std::stod(std::string(optarg));
                break;
            case 'a':
                c_arap = std::stod(std::string(optarg));
                break;
            case 'c':
                c_overhang = std::stod(std::string(optarg));
                break;
            case 'e':
                c_intersect = std::stod(std::string(optarg));
                break;
            case '?':
            default:
                std::cout<<R"(
                    usage ./support_reduction
                        -d <data_dir>
                        -f <filename>
                        -b <n_fixed_bones>
                        -i <pso_iters>
                        -p <pso_population>
                        -r <rotation_angle>
                        -a <c_arap>
                        -c <c_overhang>
                        -e <c_intersect>
                )";
                return 1;
        }
    }

    mtr_init("trace.json");

    const auto getfilepath = [&](const string& name, const string& ext){ 
        return data_dir + name + "." + ext; };

    is3d = (filename.find("woody") == 0 || filename.find("thin") == 0) 
        ? false : true;


    if (is3d) igl::readMESH(getfilepath(filename, "mesh"), V, Tet, F);
    else      igl::readOBJ(getfilepath(filename, "obj"), V, F);

    igl::readDMAT(getfilepath(filename, "dmat"), W);
    igl::normalize_row_sums(W, W);  // normalization before LBS !!
    igl::readTGF(getfilepath(filename, "tgf"), C, BE);


    {   // map vertex position to first quadrant
        RowVector3d min = V.colwise().minCoeff();
        V = V.rowwise() - min;
        C = C.rowwise() - min;
    }

    U = V;

    // find row indices to BE to be fixed
    std::vector<int> fixed_bones;
    if (BE.rows() != 0 && n_fixed_bones != 0) {
        assert(n_fixed_bones <= BE.rows());
        std::vector<double> proj_dist(BE.rows());
        for (int i = 0; i < BE.rows(); ++i) {
            proj_dist[i] = ((C.row(BE(i, 0)) + C.row(BE(i, 1))) / 2).dot(dp);
        }

        fixed_bones.resize(BE.rows());
        std::iota(fixed_bones.begin(), fixed_bones.end(), 0);
        std::partial_sort(fixed_bones.begin(), fixed_bones.begin()+n_fixed_bones, fixed_bones.end(),
            [&proj_dist](int i, int j) { return proj_dist[i] < proj_dist[j]; });
        fixed_bones.resize(n_fixed_bones);
    }

    Eigen::MatrixXd P;
    Eigen::MatrixXi G;
    support_polygon(V, 1, P, G);
    P.rowwise() -= RowVector3d(0, (V.maxCoeff() - V.minCoeff())*0.05, 0);


    ReduceSupportConfig<double> config;
    config.is3d = is3d;
    config.alpha_max = 0.25 * M_PI;
    config.dp = dp;
    config.rotation_angle = rotation_angle * M_PI / 180;
    config.fixed_bones = fixed_bones;
    config.pso_iters = pso_iters;
    config.pso_population = pso_population;
    config.c_arap = c_arap;
    config.c_overhang = c_overhang;
    config.c_intersect = c_intersect;
    config.display = true;


    //////////////////////////////////////////////////////////////////////
    //      Viewer
    //////////////////////////////////////////////////////////////////////

    igl::opengl::glfw::Viewer viewer;

    //  obj_id       handle to mesh for `.obj`
    //  ground_id   handle to convex hull of mesh
    int obj_id, ground_id;
    {
        // clear all ViewerData
        viewer.selected_data_index = viewer.data_list.size()-1;
        while(viewer.erase_mesh(viewer.selected_data_index)){};
        viewer.data().clear();
        
        obj_id = viewer.append_mesh();
        viewer.data().set_mesh(U, F);
        ground_id = viewer.append_mesh();
        viewer.data().set_mesh(P, G);
        viewer.selected_data_index = viewer.mesh_index(obj_id);
    }
    const auto draw_ground = [&](igl::opengl::glfw::Viewer &viewer) {
        Vector3d center;
        igl::centroid(U, F, center);

        // shift-down by some `offset` for better visualization
        center(1) = P.col(1).minCoeff();
        
        viewer.selected_data_index = viewer.mesh_index(ground_id);
        viewer.data().clear();
        viewer.data().set_mesh(P, G);
        viewer.data().add_points(center.transpose(), ::red);
        viewer.selected_data_index = viewer.mesh_index(obj_id);
    };

    const auto set_color = [&](igl::opengl::glfw::Viewer &viewer) {
        Eigen::MatrixXd C;
        igl::jet(W.col(selected).eval(),true,C);
        viewer.data().set_colors(C);
    };
    const auto draw_coordsys = [&](igl::opengl::glfw::Viewer &viewer, const Eigen::MatrixXd& V) {
        Eigen::Vector3d min = V.colwise().minCoeff();
        Eigen::Vector3d max = V.colwise().maxCoeff();
        Eigen::MatrixXd VCoord(7, 3);
        VCoord << 0, 0, 0,
                min(0), 0, 0,
                max(0), 0, 0,
                0, min(1), 0,
                0, max(1), 0,
                0, 0, min(2),
                0, 0, max(2);
        viewer.data().add_points(VCoord, ::green);
        for (int i = 1; i < VCoord.rows(); ++i) {
            viewer.data().add_edges(VCoord.row(0), VCoord.row(i), ::green);
        }
    };
    const auto draw_fixed_bones = [](
        igl::opengl::glfw::Viewer &viewer, const Eigen::MatrixXd& C, const Eigen::MatrixXi& BE, std::vector<int>& fixed_bones) {
        for (int i = 0; i < fixed_bones.size(); ++i) {
            auto edge = BE.row(fixed_bones[i]);
            viewer.data().add_edges(C.row(edge(0)), C.row(edge(1)), ::red);
        }
    };
    const auto draw_bones = [](
            igl::opengl::glfw::Viewer &viewer, const Eigen::MatrixXd& C, const Eigen::MatrixXi& BE) {
        viewer.data().add_points(C, ::sea_green);
        for (int i = 0; i < BE.rows(); ++i) {
            viewer.data().add_edges(C.row(BE(i, 0)), C.row(BE(i, 1)), ::sea_green);
        }
    };

    viewer.data().show_lines = false;
    viewer.data().show_overlay_depth = false;
    viewer.data().line_width = 1000;
    viewer.data().point_size = 15;
    viewer.callback_key_down = [&](igl::opengl::glfw::Viewer &, unsigned char key, int mod) {
        switch(key) {
            case '.':
                set_color(viewer);
                selected++;
                selected = std::min(std::max(selected,0),(int)W.cols()-1);
                break;
            case ',':
                set_color(viewer);
                selected--;
                selected = std::min(std::max(selected,0),(int)W.cols()-1);
                break;
            case ' ':
                V = U;
                reduce_support(V, Tet, F, C, BE, W, config, T, U);

                // reset mesh 
                viewer.data().clear();
                viewer.data().set_mesh(U, F);

                Eigen::MatrixXd CT;
                Eigen::MatrixXi BET;
                igl::deform_skeleton(C, BE, T, CT, BET);

                draw_ground(viewer);
                draw_bones(viewer, CT, BET);
                draw_coordsys(viewer, U);
                draw_fixed_bones(viewer, CT, BET, fixed_bones);

                igl::writeOBJ(getfilepath(filename+"_deformed_"+std::to_string(pso_iters), "obj"), U, F);
                break;              
        }
        return true;
    };;

    draw_ground(viewer);
    draw_bones(viewer, C, BE);
    draw_coordsys(viewer, U);
    draw_fixed_bones(viewer, C, BE, fixed_bones);

    std::cout<<
        "Press '.' to show next weight function.\n"<<
        "Press ',' to show previous weight function.\n"<<
        "Press [space] to start support reduction.\n";
    viewer.launch();


    viewer.launch();
    mtr_flush();
    mtr_shutdown();
}