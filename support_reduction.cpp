#include <Eigen/Core>

#include <igl/readOBJ.h>
#include <igl/readDMAT.h>
#include <igl/slice.h>
#include <igl/opengl/glfw/Viewer.h>

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
    igl::readOBJ(DATA_PATH "woody.obj", V, F);

    Eigen::MatrixXf W;
    igl::readDMAT(DATA_PATH "woody.dmat", W);


    double alpha_max = 0.25 * M_PI;
    Eigen::RowVector3f dp(0., 1., 0.);
    dp.normalize();
    int pso_iters = 100;
    int pso_populations = 20;
    Eigen::MatrixXf T;

    reduce_support(V, F, W, alpha_max, dp, pso_iters, pso_populations, T, U);

    // Plot the mesh with pseudocolors
    Eigen::MatrixXd Ud;
    Ud = U.cast<double>().eval();

    const Eigen::RowVector3d red(255./255., 0., 0.);
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(Ud, F);
    viewer.data().show_lines = false;
    viewer.launch();
}
