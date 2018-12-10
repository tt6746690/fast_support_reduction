#include <Eigen/Core>
#include <Eigen/Geometry>

#include <igl/readMESH.h>
#include <igl/readOBJ.h>
#include <igl/readDMAT.h>
#include <igl/readTGF.h>
#include <igl/slice.h>
#include <igl/jet.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/normalize_row_sums.h>
#include <igl/deform_skeleton.h>

#include "src/defs.h"
#include "src/reduce_support.h"
#include "src/minitrace.h"

#include <cstdio>
#include <iostream>
#include <string>

using namespace std;
using namespace Eigen;

int main(int argc, char*argv[]) {

    using namespace Eigen;
    using namespace std;
    mtr_init("build/trace.json");

    std::string filename = "woody";
    int pso_iters = 1;
    int pso_population = 1;
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
    if (argc == 3) {
        pso_iters = std::stoi(argv[1]);
        pso_population = std::stoi(argv[2]);
    }
    if (argc == 4) {
        filename = argv[1];
        pso_iters = std::stoi(argv[2]);
        pso_population = std::stoi(argv[3]);
    }
    if (argc == 10) {
        filename = argv[1];
        pso_iters = std::stoi(argv[2]);
        pso_population = std::stoi(argv[3]);
        alpha_max = (std::stod(argv[4]) / 180.) * M_PI;
        rotation_angle = (std::stod(argv[5]) / 180.) * M_PI / 2;
        c_arap = std::stod(argv[6]);
        c_overhang = std::stod(argv[7]);
        c_intersect = std::stod(argv[8]);
        rotate_model = std::stoi(argv[9]);
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
    U = V;

    Eigen::MatrixXd W;
    igl::readDMAT(DATA_PATH+filename+".dmat", W);
    igl::normalize_row_sums(W, W); // do normalization before linear blend skinning!

    Eigen::MatrixXd C;
    Eigen::MatrixXi BE;
    igl::readTGF(DATA_PATH+filename+".tgf", C, BE);
    apply_rotation(C);

    ReduceSupportConfig<double> config;
    config.is3d = is3d;
    config.alpha_max = alpha_max;
    config.dp = Eigen::RowVector3d(0., 1., 0.);
    config.rotation_angle = rotation_angle;
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
    const auto draw_coordsys = [&](igl::opengl::glfw::Viewer &viewer, Eigen::MatrixXd V) {
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

    set_color(viewer);
    viewer.data().set_mesh(U, F);
    draw_coordsys(viewer, U);
    viewer.data().show_lines = false;
    viewer.data().show_overlay_depth = false;
    viewer.data().line_width = 20;
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
                case ' ':
                    reduce_support(V, Tet, F, C, BE, W, config, T, U);
                    V = U;
                    viewer.data().set_mesh(U, F);

                    Eigen::MatrixXd CT;
                    Eigen::MatrixXi BET;
                    igl::deform_skeleton(C, BE, T, CT, BET);
                    viewer.data().add_points(CT, sea_green);
                    viewer.data().set_edges(CT, BET, sea_green);

                    draw_coordsys(viewer, U);
                    // risky faces (overhang)
                    Eigen::MatrixXd RiskyColors(config.unsafe.rows(), 3);
                    RiskyColors.rowwise() = red;
                    viewer.data().set_edges(U, config.unsafe, RiskyColors);
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