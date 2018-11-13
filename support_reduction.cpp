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

#include <cstdio>
#include <iostream>
#include <string>

using namespace std;
using namespace Eigen;

int main(int argc, char*argv[]) {

    using namespace Eigen;
    using namespace std;

    std::string filename = "woody";
    int pso_iters = 1;
    int pso_populations = 1;

    if (argc == 2) {
        filename = argv[1];
    }
    if (argc == 3) {
        pso_iters = std::stoi(argv[1]);
        pso_populations = std::stoi(argv[2]);
    }
    if (argc == 4) {
        filename = argv[1];
        pso_iters = std::stoi(argv[2]);
        pso_populations = std::stoi(argv[3]);
    }

    Eigen::MatrixXf V, U;
    Eigen::MatrixXi F;
    Eigen::RowVector3f last_mouse;
    long sel = -1;
    igl::readOBJ(DATA_PATH+filename+".obj", V, F);
    U = V;

    Eigen::MatrixXf W;
    igl::readDMAT(DATA_PATH+filename+".dmat", W);

    Eigen::MatrixXd Cd;
    Eigen::MatrixXf C;
    Eigen::MatrixXi BE;
    igl::readTGF(DATA_PATH+filename+".tgf", Cd, BE);
    C = Cd.cast<float>();

    double alpha_max = 0.25 * M_PI;
    Eigen::RowVector3f dp(0., 1., 0.);
    dp.normalize();
    Eigen::MatrixXf T;

    reduce_support(V, F, C, BE, W, alpha_max, dp, pso_iters, pso_populations, T, U);
}