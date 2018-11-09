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

#include <iostream>


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

    Eigen::MatrixXd tL, tK;
    arap_precompute(V, F, M, tL, tK);

    // Plot the mesh with pseudocolors
    const Eigen::RowVector3d sea_green(70./255.,252./255.,167./255.);
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.data().add_points(igl::slice(V,b,1),sea_green);
    viewer.data().show_lines = false;
    cout<<
        "Press [space] to toggle animation."<<endl;
    viewer.launch();
}
