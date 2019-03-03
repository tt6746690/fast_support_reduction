#include "clara.hpp"

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

#include <cstdio>
#include <iostream>
#include <string>
#include <algorithm>
#include <numeric>
#include <vector>
#include <cmath>

using namespace std;
using namespace Eigen;
using namespace clara;


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

    auto parser
        = Help(show_help)
        | Opt(data_dir, "data_dir").optional()
            ["-d"]["--data_dir"]
            ("Data Directory containing mesh/bone/weights etc.")
        | Opt(filename, "filename").optional()
            ["-f"]["--filename"]
            ("Filename under data/. `filename{.mesh, .obj, .dmat, .tgf}` required")
        | Opt(n_fixed_bones, "n_fixed_bones").optional()
            ["-b"]["--n_fixed_bones"]
            ("Number of bones to be fixed, from bottom up ")
        | Opt(pso_iters, "pso_iters").optional()
            ["-i"]["--pso_iters"]
            ("Number of particle swarm optimization iterations")
        | Opt(pso_population, "pso_population").optional()
            ["-p"]["--pso_population"]
            ("Size of particle swarm optimization population")
        | Opt(rotation_angle, "rotation_angle").optional()
            ["-r"]["--rotation_angle"]
            ("Maximum rotation (in degrees) of bones allowed")
        | Opt(c_arap, "c_arap").optional()
            ["--c_arap"]
            ("Coefficient for as-rigid-as-possible energy")
        | Opt(c_overhang, "c_overhang").optional()
            ["--c_overhang"]
            ("Coefficient for overhanging energy")
        | Opt(c_intersect, "c_intersect").optional()
            ["--c_intersect"]
            ("Coefficient for self-intersection energy");


    auto result = parser.parse(clara::Args(argc, argv));
    if (!result) { cerr<<"Error in command line: "<<result.errorMessage()<<'\n'; exit(1); }
    if (show_help) { cout<<parser; exit(0); };

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


    ReduceSupportConfig<double> config;
    config.is3d = is3d;
    config.alpha_max = 0.25 * M_PI;
    config.dp = dp;
    config.rotation_angle = rotation_angle;
    config.fixed_bones = fixed_bones;
    config.pso_iters = pso_iters;
    config.pso_population = pso_population;
    config.c_arap = c_arap;
    config.c_overhang = c_overhang;
    config.c_intersect = c_intersect;
    config.display = true;

    // compute support polygon
    // extract fixed vertices
    vector<int> minVertices;
    double tolerance = (V.col(1).maxCoeff() - V.col(1).minCoeff()) * 0.02;
    double min_Y = V.col(1).minCoeff();
    for (int i = 0; i < V.rows(); i++) {
        if (abs(V(i, 1) - min_Y) < tolerance) {
            minVertices.push_back(i);
        }
    }
    // flatten
    MatrixXd W_F;
    W_F.resize(minVertices.size(), 3);
    for (int i = 0; i < minVertices.size(); i++) {
        W_F.block(i, 0, 1, 3) = V.row(minVertices[i]);
    }
    W_F.col(1).setConstant(min_Y);
    MatrixXd P;
    MatrixXi G;
    igl::copyleft::cgal::convex_hull(W_F, P, G);


    reduce_support(V, Tet, F, C, BE, W, P, G, config, T, U);


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

    const auto draw_risky = [&config](igl::opengl::glfw::Viewer &viewer, const Eigen::MatrixXd& V) {
        for (int i = 0; i < config.unsafe.rows(); ++i) {
            viewer.data().add_edges(V.row(config.unsafe(i, 0)), V.row(config.unsafe(i, 1)), ::red);
        }
    };

    // visualize the deformed mesh
    igl::opengl::glfw::Viewer viewer;

    viewer.data().show_lines = false;
    viewer.data().show_overlay_depth = false;
    viewer.data().line_width = 1000;
    viewer.data().point_size = 15;

    // draw_coordsys(viewer, U);

    // deformed bones
    MatrixXd CT;
    MatrixXi BET;
    igl::deform_skeleton(C, BE, T, CT, BET);
    draw_bones(viewer, CT, BET);
    draw_fixed_bones(viewer, CT, BET, fixed_bones);


    double offset = (U.maxCoeff() - U.minCoeff())*0.05; // just for better visualization
    P.rowwise() -= RowVector3d(0, offset, 0);

    // compute projected centroid
    Vector3d center;
    double volume;
    igl::centroid(U, F, center, volume);
    center(1) = W_F.col(1).minCoeff() - offset;
    viewer.data().add_points(center.transpose(), ::red);


    // visualize the deformed mesh and support polygon
    // Concatenate (U, F) and (P, G) into (V_cat, F_cat)
    Eigen::MatrixXd U_cat(U.rows()+P.rows(), U.cols());
    U_cat << U, P;
    Eigen::MatrixXi F_cat(F.rows()+G.rows(), F.cols());
    F_cat << F, (G.array()+U.rows());

    viewer.data().set_mesh(U_cat, F_cat);


    viewer.launch();
    mtr_flush();
    mtr_shutdown();
}