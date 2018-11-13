#include <igl/partition.h>
#include <igl/centroid.h>
#include <igl/lbs_matrix.h>
#include <igl/pso.h>
#include <igl/min_quad_with_fixed.h>
#include <igl/forward_kinematics.h>
#include <igl/directed_edge_parents.h>
#include <igl/deform_skeleton.h>
#include <igl/opengl/glfw/Viewer.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

#include "reduce_support.h"
#include "overhang_energy.h"
#include "arap_energy.h"
#include "self_intersection.h"

#include <iostream>

typedef
  std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >
  RotationList;

ReduceSupportConfig::ReduceSupportConfig()
:   alpha_max(0.25*M_PI), 
    dp(0.,1.,0.),
    rotation_angle(0.25*M_PI),
    pso_iters(1),
    pso_population(1),
    c_arap(1),
    c_overhang(1),
    c_intersect(1),
    display(false)
{}


// convert `X` to stacked transposed transfomration for handles
//
//  Inputs:
//      `X` 1 x 6m
//          flattened input to black-box optimization
//      C, BE, P
//          arguments for forward kinematics
//  Outputs:
//      T   (d+1)m x d
//          handle transformations
//          Note d is 3 for both 2D/3D case
void unzip(
    const Eigen::RowVectorXf X,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXi& BE,
    const Eigen::MatrixXi& P,
    Eigen::MatrixXf& T)
{
    // Construct list of relative rotations in terms of quaternion
    //      from Euler's angle

    RotationList dQ;
    Eigen::RowVector3f th;
    for (int j = 0; j < BE.rows(); ++j) {
        th = X.segment(3*j, 3);
        dQ.emplace_back(
            Eigen::AngleAxisf(th(0), Eigen::Vector3f::UnitX()) *
            Eigen::AngleAxisf(th(1), Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(th(2), Eigen::Vector3f::UnitZ())
        );
    }

    // Forward kinematics
    Eigen::MatrixXd Td;
    igl::forward_kinematics(C, BE, P, dQ, Td);
    T = Td.cast<float>().eval();
}


float reduce_support(
    const Eigen::MatrixXf& V,
    const Eigen::MatrixXi& F,
    const Eigen::MatrixXf& C,
    const Eigen::MatrixXi& BE,
    const Eigen::MatrixXf& W,
    const ReduceSupportConfig& config,
    Eigen::MatrixXf& T,
    Eigen::MatrixXf& U)
{
    // Setup

    int m = W.cols();
    int d = V.cols();

    U = V;
    T.resize((d+1)*m, d);

    Eigen::MatrixXd Cd;
    Cd = C.cast<double>().eval();

    // ARAP
    Eigen::MatrixXd Vd = V.cast<double>().eval();
    Eigen::MatrixXd Wd = W.cast<double>().eval();

    // Cluster according to weights i.e. i-th vertex is in group `G(i)`
    Eigen::VectorXi G;
    {
        Eigen::VectorXi S;
        Eigen::VectorXd D;
        int n_groups = 50;
        igl::partition(Wd, n_groups, G, S, D);
    }

    Eigen::MatrixXd Md;
    igl::lbs_matrix(Vd, Wd, Md);
    Eigen::MatrixXf M;
    M = Md.cast<float>().eval();

    Eigen::SparseMatrix<float> L, K;
    arap_precompute(V, F, M, L, K);

    // Overlap
    double tau = std::sin(config.alpha_max);

    // Retrieve parents for forward kinematics
    Eigen::MatrixXi P;
    igl::directed_edge_parents(BE, P);

    // Initialize initial guess `X` and bounds `LB`, `UB`

    bool is3d = !(V.cols() == 3 && V.col(2).sum() == 0.);
    int dim = d*m;
    Eigen::RowVectorXf X(dim), LB(dim), UB(dim);

    // find joint locations
    std::vector<Eigen::Vector3d> dT(BE.rows());
    for (int i = 0; i < BE.rows(); ++i) {
        dT[i] = C.row(BE.coeff(P(i) == -1 ? 0 : P(i), 1) - 1).cast<double>().transpose();
    }

    int k = 0;

    for (int j = 0; j < m; ++j) {

        k = d*j;

        // rotation 
        X(k)   = 0;
        X(k+1) = 0;
        X(k+2) = 0;

        if (is3d) {
            LB(k)   = -config.rotation_angle;
            LB(k+1) = -config.rotation_angle;
            LB(k+2) = -config.rotation_angle;
            UB(k)   =  config.rotation_angle;
            UB(k+1) =  config.rotation_angle;
            UB(k+2) =  config.rotation_angle;
        } else {
            // 2D rotation on {x,y}-plane amounts to 
            //      fixing rotation around {x,y}-axis, and allow rotation around z-axis
            LB(k)   = 0;
            LB(k+1) = 0;
            LB(k+2) = -config.rotation_angle;
            UB(k)   = 0;
            UB(k+1) = 0;
            UB(k+2) =  config.rotation_angle;
        }
    }
    
    // Energy function

    int iter = 0;
    const std::function<float(Eigen::RowVectorXf&)> f =
       [&iter, &config,
        &Cd, &BE, &P,                               // forward kinematics
        &T, &F, &U,                                 // mesh
        &M, &L, &K,                                 // arap
        &tau, &is3d                                 // overhang
    ](Eigen::RowVectorXf & X) -> float {

        unzip(X, Cd, BE, P, T);
        U = M*T;

        double E_arap, E_overhang, E_intersect;

        std::vector<int> unsafe;
        E_overhang = overhang_energy(U, F, config.dp, tau, is3d?3:2, unsafe);

        E_arap = arap_energy(T, M, L, K);
        E_intersect = 1;

        iter += 1;
        float fX = (float) (
            config.c_arap * E_arap +  
            config.c_overhang * E_overhang + 
            config.c_intersect * E_intersect
        );

        std::cout<<"["<<iter<<"] f(X): "<<fX<<
            "\t\t("<<config.c_arap*E_arap<<", "<<
            config.c_overhang*E_overhang<<", "<<
            config.c_intersect*E_intersect<<")\n";

        return fX;
    };

    // Optimization

    auto fX = igl::pso(f, LB, UB, config.pso_iters, config.pso_population, X);
    unzip(X, Cd, BE, P, T);
    U = M * T;

    if (!config.display) {
        return fX;
    }

    // Plotting
    int selected = 0;
    Eigen::MatrixXd Ud;
    Ud = U.cast<double>().eval();
    const Eigen::RowVector3d red(1., 0., 0.);
    const Eigen::RowVector3d green(0., 1., 0.);
    const Eigen::RowVector3d blue(0., 1., 0.);
    igl::opengl::glfw::Viewer viewer;
    const auto set_color = [&](igl::opengl::glfw::Viewer &viewer) {
            Eigen::MatrixXd CC;
            igl::jet(W.col(selected).eval(),true,CC);
            viewer.data().set_colors(CC);
        };
    set_color(viewer);
    viewer.data().set_mesh(Ud, F);
    Eigen::MatrixXd CT;
    Eigen::MatrixXi BET;
    igl::deform_skeleton(Cd, BE, T.cast<double>().eval(), CT, BET);
    viewer.data().add_points(CT, red);        // joint
    viewer.data().set_edges(CT, BET, red);    // bone
    for (int i = 0; i < dT.size(); ++i) {
        viewer.data().add_edges(Eigen::RowVector3d(0,0,0), (Eigen::RowVector3d)dT[i], blue);
    }
    viewer.data().compute_normals();
    viewer.data().set_normals(viewer.data().F_normals);
    viewer.data().show_lines = false;
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
    std::cout<<
        "Press '.' to show next weight function.\n"<<
        "Press ',' to show previous weight function.\n";
    viewer.launch();

    return fX;
}