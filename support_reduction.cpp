#include <Eigen/Core>

#include <igl/readOBJ.h>
#include <igl/readDMAT.h>
#include <igl/readTGF.h>
#include <igl/slice.h>
#include <igl/snap_points.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/mat_max.h>
#include <igl/deform_skeleton.h>

#include "src/defs.h"
#include "src/reduce_support.h"

#include <iostream>

using namespace std;
using namespace Eigen;

int main(int argc, char*argv[]) {

    using namespace Eigen;
    using namespace std;

    Eigen::MatrixXf V, U;
    Eigen::MatrixXi F;
    Eigen::RowVector3f last_mouse;
    long sel = -1;
    igl::readOBJ(DATA_PATH "woody.obj", V, F);
    U = V;

    Eigen::MatrixXf W;
    igl::readDMAT(DATA_PATH "woody.dmat", W);

    Eigen::MatrixXd Cd;
    Eigen::MatrixXf C;
    Eigen::MatrixXi BE;
    igl::readTGF(DATA_PATH "woody.tgf", Cd, BE);
    C = Cd.cast<float>();


    double alpha_max = 0.25 * M_PI;
    Eigen::RowVector3f dp(0., 1., 0.);
    dp.normalize();
    int pso_iters = 1;
    int pso_populations = 1;
    Eigen::MatrixXf T;

    reduce_support(V, F, C, BE, W, alpha_max, dp, pso_iters, pso_populations, T, U);

    // Plot the mesh with pseudocolors
    Eigen::MatrixXd Ud;
    Ud = U.cast<double>().eval();

    const Eigen::RowVector3d red(1., 0., 0.);
    const Eigen::RowVector3d green(0., 1., 0.);
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(Ud, F);

    // transformed bone edges
    Eigen::MatrixXd CT;
    Eigen::MatrixXi BET;
    igl::deform_skeleton(Cd, BE, T.cast<double>().eval(), CT, BET);
    viewer.data().add_points(CT, red);       // joint
    viewer.data().set_edges(CT, BET, red);    // bone

    std::cout << "W sice: " << W.middleRows(243-1, 10) << "\n";

    viewer.data().show_lines = false;
    viewer.launch();
}