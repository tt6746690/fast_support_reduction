#include <igl/partition.h>
#include <igl/centroid.h>
#include <igl/lbs_matrix.h>
#include <igl/pso.h>

#include <algorithm>
#include <cmath>
#include <functional>

#include "reduce_support.h"
#include "overhang_energy.h"
#include "arap_energy.h"

#include <iostream>

Eigen::Affine3f unzip_affine(
    const Eigen::RowVector3f& cen,
    const Eigen::RowVector3f& th)
{
    Eigen::Affine3f model = Eigen::Affine3f::Identity();
    model.translate(cen.transpose()); 
    model.rotate(
        Eigen::AngleAxisf(th(0), Eigen::Vector3f::UnitX())*
        Eigen::AngleAxisf(th(1), Eigen::Vector3f::UnitY())*
        Eigen::AngleAxisf(th(2), Eigen::Vector3f::UnitZ())
    );
    return model;
}


// convert `X` to stacked transposed transfomration for handles
//
//  Inputs:
//      `X` 1 x 6m
//          flattened input to black-box optimization
//  Outputs:
//      T   (d+1)m x d
//          handle transformations
//          Note d is 3 for both 2D/3D case
void unzip(
    const Eigen::RowVectorXf X,
    Eigen::MatrixXf& T)
{
    int m = X.size() / (2*3);
    Eigen::Affine3f t;
    Eigen::RowVector3f cen, th;
    for (int j = 0; j < m; ++j) {
        th  = X.segment(6*j  , 3);
        cen = X.segment(6*j+3, 3);
        t = unzip_affine(cen, th);
        // std::cout << "cen: " << cen << '\n';
        // std::cout << "th: " << th << '\n';
        // std::cout << "transform: " << t.matrix() << '\n';
        T.block(4*j, 0, 4, 3) = t.matrix().block(0,0,3,4).transpose();
    }
}


float reduce_support(
    const Eigen::MatrixXf& V,
    const Eigen::MatrixXi& F,
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

    // Initialize initial guess `X` and bounds `LB`, `UB`
    bool is3d = !(V.cols() == 3 && V.col(2).sum() == 0.);
    int dim = (2*d)*m;
    Eigen::RowVectorXf X(dim), LB(dim), UB(dim);

    Eigen::RowVector3f max_coord, min_coord;
    min_coord = V.colwise().minCoeff();
    max_coord = V.colwise().maxCoeff();

    int k = 0;
    for (int j = 0; j < m; ++j) {

        k = 2*d*j;

        // rotation 
        X(k)   = 0;
        X(k+1) = 0;
        X(k+2) = 0;

        if (is3d) {
            LB(k)   = -M_PI;
            LB(k+1) = -M_PI;
            LB(k+2) = -M_PI;
            UB(k)   = M_PI;
            UB(k+1) = M_PI;
            UB(k+2) = M_PI;
        } else {
            // 2D rotation on {x,y}-plane amounts to 
            //      fixing rotation around {x,y}-axis, and allow rotation around z-axis
            LB(k)   = 0;
            LB(k+1) = 0;
            LB(k+2) = -M_PI;
            UB(k)   = 0;
            UB(k+1) = 0;
            UB(k+2) = M_PI;
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
        &T, &F, &U,                                 // mesh
        &M, &L, &K,                                 // arap
        &tau, &dp, &is3d                            // overhang
    ](Eigen::RowVectorXf & X) -> float {

        unzip(X, T);
        U = M*T;

        // std::cout << "U: " << U.topRows(5) << '\n';

        double E_arap, E_overhang;

        E_overhang = overhang_energy(U, F, dp, tau, is3d?3:2, false);
        E_arap = arap_energy(T, M, L, K);

        iter += 1;

        float fX = (float) (E_arap + E_overhang);
        std::cout << "iter: " << iter << "; f(X) = " << fX <<
            "Earap: " << E_arap << " Eoverhang: " << E_overhang << '\n';
        return fX;
    };

    // Optimization

    auto fX = igl::pso(f, LB, UB, pso_iters, pso_population, X);
    unzip(X, T);
    U = M * T;

    return fX;
}