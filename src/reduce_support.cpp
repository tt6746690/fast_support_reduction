#include <igl/partition.h>
#include <igl/centroid.h>
#include <igl/lbs_matrix.h>
#include <igl/pso.h>
#include <igl/min_quad_with_fixed.h>
#include <igl/forward_kinematics.h>
#include <igl/directed_edge_parents.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

#include "reduce_support.h"
#include "overhang_energy.h"
#include "arap_energy.h"

#include <iostream>

typedef
  std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >
  RotationList;



// convert `X` to stacked transposed transfomration for handles
//
//  Inputs:
//      `X` 1 x 6m
//          flattened input to black-box optimization
//      dT
//          joint vertices for forward kinematics
//      P
//          parants for bone edges for forward kinematics
//  Outputs:
//      T   (d+1)m x d
//          handle transformations
//          Note d is 3 for both 2D/3D case
void unzip(
    const Eigen::RowVectorXf X,
    const std::vector<Eigen::Vector3d>& dT,
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
        th = X.segment(6*j, 3);
        dQ.emplace_back(
            Eigen::AngleAxisf(th(0), Eigen::Vector3f::UnitX()) *
            Eigen::AngleAxisf(th(1), Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(th(2), Eigen::Vector3f::UnitZ())
        );
    }

    // Forward kinematics
    Eigen::MatrixXd Td;
    igl::forward_kinematics(C, BE, P, dQ, dT, Td);
    T = Td.cast<float>().eval();
}


float reduce_support(
    const Eigen::MatrixXf& V,
    const Eigen::MatrixXi& F,
    const Eigen::MatrixXf& C,
    const Eigen::MatrixXi& BE,
    const Eigen::MatrixXf& W,
    double alpha_max,
    const Eigen::RowVector3f& dp,
    int pso_iters,
    int pso_population,
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
    double tau = std::sin(alpha_max);

    // Retrieve parents for forward kinematics
    Eigen::MatrixXi P;
    igl::directed_edge_parents(BE, P);

    // Initialize initial guess `X` and bounds `LB`, `UB`

    bool is3d = !(V.cols() == 3 && V.col(2).sum() == 0.);
    int dim = (2*d)*m;
    Eigen::RowVectorXf X(dim), LB(dim), UB(dim);

    // find joint locations
    std::vector<Eigen::Vector3d> dT(BE.rows());
    for (int i = 0; i < BE.rows(); ++i) {
        dT[i] = (C.row(BE.coeff(P(i) == -1 ? 0 : P(i), 1)).cast<double>().transpose());
    }

    Eigen::RowVector3f max_coord, min_coord;
    min_coord = V.colwise().minCoeff();
    max_coord = V.colwise().maxCoeff();

    int k = 0;
    float psi = M_PI / 8;

    for (int j = 0; j < m; ++j) {

        k = 2*d*j;

        // rotation 
        X(k)   = 0;
        X(k+1) = 0;
        X(k+2) = 0;

        if (is3d) {
            LB(k)   = -psi;
            LB(k+1) = -psi;
            LB(k+2) = -psi;
            UB(k)   = psi;
            UB(k+1) = psi;
            UB(k+2) = psi;
        } else {
            // 2D rotation on {x,y}-plane amounts to 
            //      fixing rotation around {x,y}-axis, and allow rotation around z-axis
            LB(k)   = 0;
            LB(k+1) = 0;
            LB(k+2) = -psi;
            UB(k)   = 0;
            UB(k+1) = 0;
            UB(k+2) = psi;
        }

        k += 3;
        // translation
        X(k)   = 0;
        X(k+1) = 0;
        X(k+2) = 0;
        LB(k)   = 0;
        LB(k+1) = 0;
        LB(k+2) = 0;
        UB(k)   = 0;
        UB(k+1) = 0;
        UB(k+2) = 0;

        // LB(k)   = min_coord(0);
        // LB(k+1) = min_coord(1);
        // LB(k+2) = is3d ? min_coord(2) : 0;
        // UB(k)   = max_coord(0);
        // UB(k+1) = max_coord(1);
        // UB(k+2) = is3d ? max_coord(2) : 0;
    }
    
    // Energy function

    int iter = 0;
    const std::function<float(Eigen::RowVectorXf&)> f =
       [&iter, 
        &dT, &Cd, &BE, &P,                           // forward kinematics
        &T, &F, &U,                                 // mesh
        &M, &L, &K,                                 // arap
        &tau, &dp, &is3d                            // overhang
    ](Eigen::RowVectorXf & X) -> float {

        unzip(X, dT, Cd, BE, P, T);
        U = M*T;

        double E_arap, E_overhang;

        std::vector<int> unsafe;
        E_overhang = overhang_energy(U, F, dp, tau, is3d?3:2, unsafe);
        E_arap = arap_energy(T, M, L, K);

        iter += 1;

        float fX = (float) (0.00001*E_arap + 0.5*E_overhang);
        std::cout << "iter: " << iter << "; f(X) = " << fX <<
            "Earap: " << 0.00001*E_arap << " Eoverhang: " << E_overhang << '\n';
        return fX;
    };

    // Optimization

    auto fX = igl::pso(f, LB, UB, pso_iters, pso_population, X);
    unzip(X, dT, Cd, BE, P, T);
    U = M * T;

    return fX;
}