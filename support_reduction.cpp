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
    int pso_iters = 2;
    int pso_populations = 5;
    Eigen::MatrixXf T;

    reduce_support(V, F, C, BE, W, alpha_max, dp, pso_iters, pso_populations, T, U);
}