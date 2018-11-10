#include <Eigen/Core>

#include <igl/readOBJ.h>
#include <igl/readDMAT.h>
#include <igl/lbs_matrix.h>
#include <igl/partition.h>
#include <igl/mat_max.h>
#include <igl/slice.h>
#include <igl/opengl/glfw/Viewer.h>

#include "src/defs.h"
#include "src/compute_bbw.h"
#include "src/arap_energy.h"
#include "src/overhang_energy.h"

#include <iostream>
#include <cmath>


int main(int argc, char*argv[]) {

    using namespace Eigen;
    using namespace std;

    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    igl::readOBJ(DATA_PATH "woody.obj", V, F);

    // #b x m, where `m` is number of bones
    Eigen::MatrixXd W, M;
    igl::readDMAT(DATA_PATH "woody.dmat", W);
    igl::lbs_matrix(V, W, M);

    // Cluster according to weights
    // i.e. i-th vertex is in group `G(i)`
    Eigen::VectorXi G;
    {
        VectorXi S;
        VectorXd D;
        int n_groups = 50;
        igl::partition(W, n_groups, G, S, D);
    }

    // vertices corresponding to handles (those with maximum weight)
    Eigen::VectorXi b;
    {
        VectorXd maxW;
        igl::mat_max(W, 1, maxW, b);
    }

    Eigen::SparseMatrix<double> L, K;
    arap_precompute(V, F, M, L, K);

    Eigen::RowVector3d dp(0., 1., 0.);
    double alpha_max = 0.25 * M_PI;
    double tau = std::sin(alpha_max);
    auto e = overhang_energy(V, F, dp, tau, true);
}
