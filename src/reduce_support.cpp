#include <igl/partition.h>
#include <igl/centroid.h>
#include <igl/lbs_matrix.h>
#include <igl/pso.h>
#include <igl/min_quad_with_fixed.h>
#include <igl/forward_kinematics.h>
#include <igl/directed_edge_parents.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/boundary_loop.h>

#include <omp.h>

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

#include "reduce_support.h"
#include "overhang_energy.h"
#include "arap_energy.h"
#include "self_intersection.h"
#include "minitrace.h"

#include <iostream>

typedef
  std::vector<Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond> >
  RotationList;



// convert `X` to stacked transposed transfomration for handles
//
//  Inputs:
//      `X` 1 x 6m
//          flattened input to black-box optimization
//       C, BE, P
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

        /*
        // this stops VS2017 from complaining about mixing numeric types
        Eigen::Quaternionf qf = Eigen::AngleAxisf(th(0), Eigen::Vector3f::UnitX()) *
            Eigen::AngleAxisf(th(1), Eigen::Vector3f::UnitY()) *
            Eigen::AngleAxisf(th(2), Eigen::Vector3f::UnitZ());
        Eigen::Quaterniond qd = qf.cast<double>();
        dQ.emplace_back(qd);
        */

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
    const Eigen::MatrixXi& Tet,
    const Eigen::MatrixXi& F,
    const Eigen::MatrixXf& C,
    const Eigen::MatrixXi& BE,
    const Eigen::MatrixXf& W,
    ReduceSupportConfig<float>& config,
    Eigen::MatrixXf& T,
    Eigen::MatrixXf& U)
{
    // Setup

    int m = W.cols();   // number of bones
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

    // Overhang
    double tau = std::cos(config.alpha_max);
    Eigen::VectorXi bnd;
    igl::boundary_loop(F, bnd);

    // Retrieve parents for forward kinematics
    Eigen::MatrixXi P;
    igl::directed_edge_parents(BE, P);

    // Initialize initial guess `X` and bounds `LB`, `UB`
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

        if (std::find(config.fixed_bones.begin(), config.fixed_bones.end(), j) != config.fixed_bones.end()) {
            for (int i = 0; i < 3; ++i) {
                LB(k+i) = 0;
                UB(k+i) = 0;
            }
        } else {
            if (config.is3d) {
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

    }
    
    // Energy function

    int iter = 0;
    const std::function<float(Eigen::RowVectorXf&)> f =
       [&iter, &config, &V,
        &Cd, &BE, &P,                               // forward kinematics
        &T, &F, &U, &Tet,                           // mesh
        &M, &L, &K,                                 // arap
        &tau, &bnd                                  // overhang
    ](Eigen::RowVectorXf & X) -> float {

        MTR_SCOPE_FUNC();

        unzip(X, Cd, BE, P, T);
        U = M * T;

        double E_arap, E_overhang, E_intersect;

        if (config.is3d) {
            E_overhang = overhang_energy_3d(U, F,   config.dp, tau);
            E_intersect = self_intersection_3d(U, F);
        } else {
            E_overhang = overhang_energy_2d(U, bnd, config.dp, tau, config.unsafe);
            E_intersect = self_intersection_2d(U, F);
        }
        
        E_arap = arap_energy(V, T, M, config.is3d?Tet:F, L, K, config.is3d);

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
    float fX;
    #pragma omp parallel
    {
        fX = igl::pso(f, LB, UB, config.pso_iters, config.pso_population, X);
    }

    std::cout << "final fX: " << fX << '\n';
    unzip(X, Cd, BE, P, T);
    U = M * T;
    overhang_energy_risky(U, F, config.dp, tau, config.unsafe);

    return fX;
}




double reduce_support(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& Tet,
    const Eigen::MatrixXi& F,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXi& BE,
    const Eigen::MatrixXd& W,
    ReduceSupportConfig<double>& config,
    Eigen::MatrixXd& T,
    Eigen::MatrixXd& U)
{
    Eigen::MatrixXf Vf = V.cast<float>();
    Eigen::MatrixXf Cf = C.cast<float>();
    Eigen::MatrixXf Wf = W.cast<float>();
    Eigen::MatrixXf Tf = T.cast<float>();
    Eigen::MatrixXf Uf = U.cast<float>();
    ReduceSupportConfig<float> configf(config);
    float fX = reduce_support(Vf, Tet, F, Cf, BE, Wf, configf, Tf, Uf);
    config = configf;
    T = Tf.cast<double>();
    U = Uf.cast<double>();
    return (double) fX;
}