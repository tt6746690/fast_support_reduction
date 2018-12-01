#include <Eigen/Core>

#include <igl/boundary_conditions.h>
#include <igl/jet.h>
#include <igl/normalize_row_sums.h>
#include <igl/readDMAT.h>
#include <igl/writeDMAT.h>
#include <igl/readMESH.h>
#include <igl/readTGF.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/bbw.h>
#include <igl/copyleft/tetgen/tetrahedralize.h>


#include "src/defs.h"
#include <string>

int main(int argc, char*argv[]) {
    
    using namespace Eigen;
    using namespace std;

    std::string filename = "hand";
    if (argc == 2) {
        filename = argv[1];
    }

    // (V, F) triangle mesh (volume) for 2D surface object
    // (C, BE) joint vertex location and bone edges
    Eigen::MatrixXd V, C;
    Eigen::MatrixXi F, T, BE;


    // igl::readMESH(DATA_PATH+filename+".mesh",V,T,F);
    igl::readOBJ(DATA_PATH+filename+".obj", V, F);

    igl::readTGF(DATA_PATH+filename+".tgf", C, BE);

    Eigen::MatrixXd TV;
    Eigen::MatrixXi TF, TT;
    igl::copyleft::tetgen::tetrahedralize(V, F, "pq1.414a0.01", TV, TT, TF);




    // Compute bbw

    // b: List of boundary indices (fixed value indices into V)
    // bc: List of boundary conditions of each weight function
    Eigen::VectorXi b;
    Eigen::MatrixXd bc;
    bool result = igl::boundary_conditions(V, F, C, VectorXi(), BE, MatrixXi(), b, bc);
    if (!result)
        printf("boundary_conditions result suspicious\n");

    // Compute weight matrix
    int selected = 0;
    Eigen::MatrixXd W;
    igl::BBWData bbw_data;
    bbw_data.active_set_params.max_iter = 30;
    bbw_data.verbosity = 2;
    if (!igl::bbw(V, F, b, bc, bbw_data, W)) {
        printf("Failed to compute bbw\n");
    }
    // enforce parity of unity constraint
    igl::normalize_row_sums(W, W);
    // outputs bbw weights
    igl::writeDMAT(DATA_PATH+filename+".dmat", W, true);

    bool display = true;
    if (display) {
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
    }
}
