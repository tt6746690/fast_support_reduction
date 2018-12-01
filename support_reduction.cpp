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

    auto apply_rotation = [rotate_model](Eigen::MatrixXf& V) {
        Eigen::Transform<float,3,Eigen::Affine> t(
            Eigen::AngleAxisf((rotate_model==1)?-M_PI/2:0, Eigen::Vector3f::UnitX()) *
            Eigen::AngleAxisf((rotate_model==2)?-M_PI/2:0, Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf((rotate_model==3)?-M_PI/2:0, Eigen::Vector3f::UnitZ())
        );
        
        if (rotate_model != 0) {
            Eigen::Vector3f p;
            for (int i = 0; i < V.rows(); ++i) {
                p = V.row(i).transpose();
                V.row(i) = t * p;
            }
        }
    };

    bool is3d = (filename.find("woody") == 0 || filename.find("thin") == 0) ? 
        false : true;

    Eigen::MatrixXf V, U;
    Eigen::MatrixXi Tet, F;   // Tet=tetrahedron mesh

    if (is3d) {
        igl::readMESH(DATA_PATH+filename+".mesh", V, Tet, F);
    } else {
        igl::readOBJ(DATA_PATH+filename+".obj", V, F);
    }
    apply_rotation(V);
    U = V;

    Eigen::MatrixXf W;
    igl::readDMAT(DATA_PATH+filename+".dmat", W);
    // do normalization before lbs
    igl::normalize_row_sums(W, W);

    Eigen::MatrixXd Cd;
    Eigen::MatrixXf C;
    Eigen::MatrixXi BE;
    igl::readTGF(DATA_PATH+filename+".tgf", Cd, BE);
    C = Cd.cast<float>();
    apply_rotation(C);

    ReduceSupportConfig config;
    config.is3d = is3d;
    config.alpha_max = alpha_max;
    config.dp = Eigen::RowVector3f(0., 1., 0.);
    config.rotation_angle = rotation_angle;
    config.pso_iters = pso_iters;
    config.pso_population = pso_population;
    config.c_arap = c_arap;
    config.c_overhang = c_overhang;
    config.c_intersect = c_intersect;
    config.display = true;

    Eigen::MatrixXf T;
    reduce_support(V, Tet, F, C, BE, W, config, T, U);

    mtr_flush();
    mtr_shutdown();
}