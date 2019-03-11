#include <igl/partition.h>
#include <igl/centroid.h>
#include <igl/lbs_matrix.h>
#include <igl/pso.h>
#include <igl/min_quad_with_fixed.h>
#include <igl/forward_kinematics.h>
#include <igl/directed_edge_parents.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/boundary_loop.h>
#include <igl/winding_number.h>
#include <igl/centroid.h>
#include <igl/cotmatrix.h>
#include <igl/covariance_scatter_matrix.h>
#include <igl/mode.h>
#include <igl/group_sum_matrix.h>
#include <igl/repdiag.h>
#include <igl/arap_rhs.h>

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

#include "intersection_volume.h"

#include <igl/opengl/glfw/background_window.h>

#include <iostream>

const int ren_width  = 400;
const int ren_height = 400;

const double stand_energy = 1e6;

std::string shader_dir = "../src/shaders/";

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
    const Eigen::MatrixXf& PO,
    const Eigen::MatrixXi& G,
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

    // // Cluster according to weights i.e. i-th vertex is in group `G(i)`
    // Eigen::VectorXi G;
    // {
    //     Eigen::VectorXi S;
    //     Eigen::VectorXd D;
    //     int n_groups = 50;
    //     igl::partition(Wd, n_groups, G, S, D);
    // }



    // arap precompute: need to be moved to a seperate file later
    // --------------------------------------------------------------------------------------------

    Eigen::MatrixXd Md;
    igl::lbs_matrix(Vd, Wd, Md);
    Eigen::MatrixXf M;
    M = Md.cast<float>().eval();

    Eigen::SparseMatrix<double> Ld, CSMd, Kd;
    igl::cotmatrix(Vd, Tet, Ld);
    // igl::arap_linear_block(Vd, Tet, 0, igl::ARAP_ENERGY_TYPE_ELEMENTS, CSM);
    igl::covariance_scatter_matrix(Vd, Tet, igl::ARAP_ENERGY_TYPE_ELEMENTS, CSMd);
    igl::arap_rhs(Vd, Tet, 3, igl::ARAP_ENERGY_TYPE_ELEMENTS, Kd);

    // Get group sum scatter matrix, when applied sums all entries of the same
    // group according to G
    // Cluster according to weights
    Eigen::VectorXi Gr;
    {
        Eigen::VectorXi S;
        Eigen::VectorXd D;
        igl::partition(Wd, 20, Gr, S, D);
    }
    Eigen::SparseMatrix<double> G_sum;
    Eigen::Matrix<int, Eigen::Dynamic, 1> GG;
    Eigen::MatrixXi GF(Tet.rows(), Tet.cols());
    for(int i = 0; i < Tet.cols(); i++) {
        Eigen::Matrix<int, Eigen::Dynamic, 1> GFi;
        igl::slice(Gr, Tet.col(i), GFi);
        GF.col(i) = GFi;
    }
    igl::mode<int>(GF, 2, GG);
    Gr = GG;
    igl::group_sum_matrix(Gr, G_sum);
    Eigen::SparseMatrix<double> G_sum_dim;
    igl::repdiag(G_sum, 3, G_sum_dim);
    CSMd = (G_sum_dim * CSMd).eval();

    Eigen::SparseMatrix<float> K;
    K = Kd.cast<float>().eval();

    Eigen::SparseMatrix<float> CSM;
    CSM = CSMd.cast<float>().eval();

    Eigen::SparseMatrix<float> L;
    L = Ld.cast<float>().eval();

    // ----------------------------------------------------------------------------------------------




    // fast self intersection
    SelfIntersectionVolume vol(V, W, M, F, ren_width, ren_height, shader_dir);
    GLFWwindow* window;
    igl::opengl::glfw::background_window(window); // setup window and context
    vol.prepare();

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
        &M, &L, &K, &CSM, &Gr,                               // arap
        &tau, &bnd,                                 // overhang
        &vol,                                       // fast self intersection
        &PO, &G                                     // make it stand
    ](Eigen::RowVectorXf & X) -> float {

        unzip(X, Cd, BE, P, T);
        U = M * T;

        double E_arap, E_overhang, E_intersect, E_stand;

        // compute projected centroid
        Eigen::Vector3f center;
        float volume;
        igl::centroid(U, F, center, volume);
        center(1) = U.col(1).minCoeff();
        double wind = igl::winding_number(PO, G, center); // winding number to check inside or outside

        // stand energy
        if (wind > 0.49) { /// just in case
            E_stand = 0;
            Eigen::MatrixXf Tp;
            Tp.resize(T.cols()+1, T.rows());
            Eigen::MatrixXf row4;
            row4.resize(1, T.rows());
            for (int i = 0; i < Tp.cols()/4; i++) {
                row4.block(0, i*4, 1, 4) = Eigen::RowVector4f(0, 0, 0, 1);
            }
            Tp << T.transpose(), row4;
            
            Eigen::RowVector3f A_center = 0.5*(U.colwise().maxCoeff() + U.colwise().minCoeff());
            float new_half = (U.rowwise()-A_center).rowwise().norm().maxCoeff() * 1.0001;
            igl::ortho(-new_half, new_half, -new_half, new_half, 0, 2*new_half, vol.projection);
            Eigen::Vector3f eye(A_center(0), A_center(1), -new_half+A_center(2));
            Eigen::Vector3f target(A_center(0), A_center(1), A_center(2));
            Eigen::Vector3f up(0, 1, 0);
            igl::look_at(eye, target, up, vol.view.matrix());

            vol.model = Eigen::Affine3f::Identity();
            vol.ortho_box_volume = pow(2*new_half, 3.0);

            if (config.is3d) {
                E_overhang = overhang_energy_3d(U, F, config.dp, tau);
                E_intersect = vol.compute(Tp);
            } else {
                E_overhang = overhang_energy_2d(U, bnd, config.dp, tau, config.unsafe);
                E_intersect = self_intersection_2d(U, F);
            }
            
            E_arap = arap_energy(U, K, Gr, L, CSM, config.is3d);

        }
        else {
            E_stand = stand_energy;
            E_arap = 0;
            E_overhang = 0;
            E_intersect = 0;
        }


        iter += 1;
        float fX = (float) (
            config.c_arap * E_arap +  
            config.c_overhang * E_overhang + 
            config.c_intersect * E_intersect +
            E_stand
        );

        std::cout<<"["<<iter<<"] f(X): "<<fX<<
            "\t\t("<<config.c_arap*E_arap<<", "<<
            config.c_overhang*E_overhang<<", "<<
            config.c_intersect*E_intersect<<")\n";

        return fX;
    };

    // Optimization
    float fX;
    fX = igl::pso(f, LB, UB, config.pso_iters, config.pso_population, X);

    std::cout << "final fX: " << fX << '\n';

    glfwDestroyWindow(window);
    glfwTerminate();

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
    const Eigen::MatrixXd& PO,
    const Eigen::MatrixXi& G,
    ReduceSupportConfig<double>& config,
    Eigen::MatrixXd& T,
    Eigen::MatrixXd& U)
{
    Eigen::MatrixXf Vf = V.cast<float>();
    Eigen::MatrixXf Cf = C.cast<float>();
    Eigen::MatrixXf Wf = W.cast<float>();
    Eigen::MatrixXf Tf = T.cast<float>();
    Eigen::MatrixXf Uf = U.cast<float>();
    Eigen::MatrixXf POf = PO.cast<float>();
    ReduceSupportConfig<float> configf(config);
    float fX = reduce_support(Vf, Tet, F, Cf, BE, Wf, POf, G, configf, Tf, Uf);
    config = configf;
    T = Tf.cast<double>();
    U = Uf.cast<double>();
    return (double) fX;
}