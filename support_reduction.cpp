#include <Eigen/Core>
#include <Eigen/Geometry>

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

#include "src/defs.h"
#include "src/reduce_support.h"
#include "src/overhang_energy.h"
#include "src/minitrace.h"

#include <cstdio>
#include <iostream>
#include <string>
#include <algorithm>
#include <numeric>
#include <vector>
#include <cmath>

using namespace std;
using namespace Eigen;

int main(int argc, char*argv[]) {

    using namespace Eigen;
    using namespace std;
    mtr_init("build/trace.json");

    std::string filename = "woody";
    int n_fixed_bones = 0;
    int pso_iters = 1;
    int pso_population = 1;
    Eigen::RowVector3d dp = Eigen::RowVector3d(0., 1., 0.);
    double c_arap = 1;
    double c_overhang = 1;
    double c_intersect = 1;
    double alpha_max = 0.25 * M_PI;     // 45
    double rotation_angle = M_PI / 10;
    int rotate_model = 0;
    //  0 - dont rotate 
    //  1 - x 
    //  2 - y
    //  3 - z

    if (argc == 2) {
        filename = argv[1];
    }
    if (argc == 11) {
        filename = argv[1];
        n_fixed_bones = std::stoi(argv[2]);
        pso_iters = std::stoi(argv[3]);
        pso_population = std::stoi(argv[4]);
        alpha_max = (std::stod(argv[5]) / 180.) * M_PI;
        rotation_angle = (std::stod(argv[6]) / 180.) * M_PI / 2;
        c_arap = std::stod(argv[7]);
        c_overhang = std::stod(argv[8]);
        c_intersect = std::stod(argv[9]);
        rotate_model = std::stoi(argv[10]);
    }

    auto apply_rotation = [rotate_model](Eigen::MatrixXd& V) {
        Eigen::Transform<double,3,Eigen::Affine> t(
            Eigen::AngleAxisd((rotate_model==1)?-M_PI/2:0, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd((rotate_model==2)?-M_PI/2:0, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd((rotate_model==3)?-M_PI/2:0, Eigen::Vector3d::UnitZ())
        );
        
        if (rotate_model != 0) {
            Eigen::Vector3d p;
            for (int i = 0; i < V.rows(); ++i) {
                p = V.row(i).transpose();
                V.row(i) = t * p;
            }
        }
    };

    bool is3d = (filename.find("woody") == 0 || filename.find("thin") == 0) ? 
        false : true;

    Eigen::MatrixXd V, U;
    Eigen::MatrixXi Tet, F;   // Tet=tetrahedron mesh

    if (is3d) {
        igl::readMESH(DATA_PATH+filename+".mesh", V, Tet, F);
    } else {
        igl::readOBJ(DATA_PATH+filename+".obj", V, F);
    }
    apply_rotation(V);

    Eigen::MatrixXd W;
    igl::readDMAT(DATA_PATH+filename+".dmat", W);
    igl::normalize_row_sums(W, W); // do normalization before linear blend skinning!

    Eigen::MatrixXd C;
    Eigen::MatrixXi BE;
    igl::readTGF(DATA_PATH+filename+".tgf", C, BE);
    apply_rotation(C);

    // move V to first quadrant
    Eigen::RowVector3d min = V.colwise().minCoeff();
    V = V.rowwise() - min;
    C = C.rowwise() - min;
    U = V;


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


    ReduceSupportConfig<double> config;
    config.is3d = is3d;
    config.alpha_max = alpha_max;
    config.dp = dp;
    config.rotation_angle = rotation_angle;
    config.fixed_bones = fixed_bones;
    config.pso_iters = pso_iters;
    config.pso_population = pso_population;
    config.c_arap = c_arap;
    config.c_overhang = c_overhang;
    config.c_intersect = c_intersect;
    config.display = true;

    Eigen::MatrixXd T;

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////


    int selected = 0;
    const Eigen::RowVector3d red(1., 0., 0.);
    const Eigen::RowVector3d green(0., 1., 0.);
    const Eigen::RowVector3d blue(0., 1., 0.);
    const Eigen::RowVector3d sea_green(70./255.,252./255.,167./255.);
    igl::opengl::glfw::Viewer viewer;
    const auto set_color = [&](igl::opengl::glfw::Viewer &viewer) {
            Eigen::MatrixXd CC;
            igl::jet(W.col(selected).eval(),true,CC);
            viewer.data().set_colors(CC);
        };
    const auto draw_coordsys = [&](igl::opengl::glfw::Viewer &viewer, const Eigen::MatrixXd& V) {
        // coordinates 
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
        viewer.data().add_points(VCoord, green);
        for (int i = 1; i < VCoord.rows(); ++i) {
            viewer.data().add_edges(VCoord.row(0), VCoord.row(i), green);
        }
    };
    const auto draw_fixed_bones = [&red](
        igl::opengl::glfw::Viewer &viewer, const Eigen::MatrixXd& C, const Eigen::MatrixXi& BE, std::vector<int>& fixed_bones) {
        for (int i = 0; i < fixed_bones.size(); ++i) {
            auto edge = BE.row(fixed_bones[i]);
            viewer.data().add_edges(C.row(edge(0)), C.row(edge(1)), red);
        }
    };
    const auto draw_bones = [&sea_green](igl::opengl::glfw::Viewer &viewer, const Eigen::MatrixXd& C, const Eigen::MatrixXi& BE) {
        viewer.data().add_points(C, sea_green);
        for (int i = 0; i < BE.rows(); ++i) {
            viewer.data().add_edges(C.row(BE(i, 0)), C.row(BE(i, 1)), sea_green);
        }
    };
    const auto draw_risky = [&config, &red](igl::opengl::glfw::Viewer &viewer, const Eigen::MatrixXd& V) {
        for (int i = 0; i < config.unsafe.rows(); ++i) {
            viewer.data().add_edges(V.row(config.unsafe(i, 0)), V.row(config.unsafe(i, 1)), red);
        }
    };

    viewer.data().set_mesh(U, F);
    draw_bones(viewer, C, BE);
    draw_coordsys(viewer, U);
    draw_fixed_bones(viewer, C, BE, fixed_bones);
    if (config.is3d) {
        overhang_energy_risky(U, F, config.dp, std::cos(config.alpha_max), config.unsafe);
        draw_risky(viewer, U);
    }
    viewer.data().show_lines = false;
    viewer.data().show_overlay_depth = false;
    viewer.data().line_width = 20;
    viewer.callback_key_down = 
        [&](igl::opengl::glfw::Viewer &, unsigned char key, int mod) {
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
                    reduce_support(V, Tet, F, C, BE, W, config, T, U);
                    V = U;

                    // clear all ViewerData
                    viewer.selected_data_index = viewer.data_list.size()-1;
                    while(viewer.erase_mesh(viewer.selected_data_index)){};
                    viewer.data().clear();

                    // reset mesh 
                    viewer.data().set_mesh(U, F);
                    draw_coordsys(viewer, U);

                    // deformed bones
                    Eigen::MatrixXd CT;
                    Eigen::MatrixXi BET;
                    igl::deform_skeleton(C, BE, T, CT, BET);
                    draw_fixed_bones(viewer, CT, BET, fixed_bones);
                    draw_bones(viewer, CT, BET);

                    draw_risky(viewer, U);

                    std::string outf = DATA_PATH+filename+"_deformed_" + std::to_string(pso_iters);
                    if (config.is3d) {
                        igl::writeOBJ(outf + ".obj", U, F);
                        igl::writeMESH(outf + ".mesh", U, Tet, F);
                    } else {
                        igl::writeOBJ(outf + ".obj", U, F);
                    }
                    break;              
            }
            return true;
        };
    std::cout<<
        "Press '.' to show next weight function.\n"<<
        "Press ',' to show previous weight function.\n";
    viewer.launch();

    ////////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////////

    mtr_flush();
    mtr_shutdown();
}