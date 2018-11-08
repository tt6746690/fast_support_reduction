// Because of Mosek complications, we don't use static library if Mosek is used.
#ifdef LIBIGL_WITH_MOSEK
    #ifdef IGL_STATIC_LIBRARY
        #undef IGL_STATIC_LIBRARY
    #endif
#endif

#ifndef DATA_PATH
    #define DATA_PATH "../data/"
#endif

#include <igl/boundary_conditions.h>
#include <igl/jet.h>
#include <igl/lbs_matrix.h>
#include <igl/normalize_row_sums.h>
#include <igl/readDMAT.h>
#include <igl/readMESH.h>
#include <igl/readTGF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/bbw.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <algorithm>
#include <iostream>

using namespace Eigen;
using namespace std;


int main(int argc, char* argv[]) {

    // (V, F) triangle mesh (volume) for 2D surface object
    // (C, BE) joint vertex location and bone edges
    Eigen::MatrixXd V, C;
    Eigen::MatrixXi F, BE;

    igl::readOBJ(DATA_PATH "woody.obj", V, F);
    igl::readTGF(DATA_PATH "woody.tgf", C, BE);

    // Instead of remeshing with additional handle positions C
    // Map handle each handle position to closest vertex in V
    for (int i = 0; i < C.rows(); ++i) {
        MatrixXd::Index closest;
        (V.rowwise() - C.row(i)).rowwise().squaredNorm().minCoeff(&closest);
        C.row(i) = V.row(closest);
    }

    // // Compute bbw

    // b: List of boundary indices (fixed value indices into V)
    // bc: List of boundary conditions of each weight function
    Eigen::VectorXi b;
    Eigen::MatrixXd bc;
    bool result = igl::boundary_conditions(V, F, C, VectorXi(), BE, MatrixXi(), b, bc);
    if (!result)
        printf("boundary_conditions result suspicious\n");

    // Compute weight matrix
    int selected = 0;
    Eigen::MatrixXd W, M;
    igl::BBWData bbw_data;
    bbw_data.active_set_params.max_iter = 30;
    bbw_data.verbosity = 2;
    if (!igl::bbw(V, F, b, bc, bbw_data, W)) {
        printf("Failed to compute bbw\n");
        return EXIT_FAILURE;
    }

    // enforce parity of unity constraint
    igl::normalize_row_sums(W, W);
    // precompute skinning matrix
    igl::lbs_matrix(V, W, M);


    // cout << "C" << C << '\n';
    // cout << "BE" << BE << '\n';
    // cout <<  "V" << V.block(0,0,10,3) << '\n';
    // cout <<  "F" << F.block(0,0,10,3) << '\n';
    // cout << "W" << W.row(0) << '\n';

    // Viewer
    const Eigen::RowVector3d sea_green(70./255.,252./255.,167./255.);
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    const auto set_color = [&](igl::opengl::glfw::Viewer &viewer) {
        Eigen::MatrixXd C;
        igl::jet(W.col(selected).eval(),true,C);
        viewer.data().set_colors(C);
    };
    set_color(viewer);
    viewer.data().set_edges(C, BE, sea_green);
    viewer.data().show_lines = false;
    viewer.data().show_overlay_depth = false;
    viewer.data().line_width = 1;
    viewer.callback_key_down = 
        [&](igl::opengl::glfw::Viewer &, unsigned char key, int mod) {
            switch(key) {
                case '.':
                    selected++;
                    selected = std::min(std::max(selected,0),(int)W.cols()-1);
                    set_color(viewer);
                    break;
                case ',':
                    selected--;
                    selected = std::min(std::max(selected,0),(int)W.cols()-1);
                    set_color(viewer);
                    break;
            }
            return true;
        };
    cout<<
    "Press '.' to show next weight function."<<endl<<
    "Press ',' to show previous weight function."<<endl<<
    viewer.launch();
    return EXIT_SUCCESS;
}
