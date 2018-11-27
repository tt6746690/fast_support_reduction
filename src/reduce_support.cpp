#include <igl/partition.h>
#include <igl/centroid.h>
#include <igl/lbs_matrix.h>
#include <igl/pso.h>
#include <igl/min_quad_with_fixed.h>
#include <igl/forward_kinematics.h>
#include <igl/directed_edge_parents.h>
#include <igl/deform_skeleton.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/mode.h>
#include <igl/slice.h>
#include <igl/group_sum_matrix.h>
#include <igl/repdiag.h>
#include <igl/covariance_scatter_matrix.h>
#include <igl/colon.h>
#include <igl/is_sparse.h>
#include <igl/polar_svd3x3.h>

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
    int m = W.cols(); // num of bones
    int d = V.cols(); // dimension --- in 2d it's still 3 tho


    std::cout << "m: " << m << std::endl;

    U = V;
    T.resize((d + 1) * m, d);

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

    Eigen::Matrix<int, Eigen::Dynamic, 1> GG;
    Eigen::MatrixXi GF(F.rows(), F.cols());
    for (int j = 0; j < F.cols(); j++) {
        Eigen::Matrix<int, Eigen::Dynamic, 1> GFj;
        igl::slice(G, F.col(j), GFj);
        GF.col(j) = GFj;
    }
    igl::mode<int>(GF, 2, GG);
    Eigen::SparseMatrix<double> G_sum;
    igl::group_sum_matrix(GG, G_sum);

    Eigen::MatrixXd Md;
    igl::lbs_matrix(Vd, Wd, Md);
    Eigen::MatrixXf M;
    M = Md.cast<float>().eval();

    Eigen::SparseMatrix<float> L, K;
    arap_precompute(V, F, M, L, K);

    Eigen::SparseMatrix<double> CSM;
    igl::covariance_scatter_matrix(Vd, F, igl::ARAP_ENERGY_TYPE_ELEMENTS, CSM);





    // // ------------ Dimension: now d------------ //
    Eigen::SparseMatrix<double> G_sum_dim;
    igl::repdiag(G_sum, d, G_sum_dim);
    CSM = (G_sum_dim * CSM).eval();


    // construct CSM_M
    Eigen::MatrixXd Mcd;
    igl::lbs_matrix_column(Vd, Wd, Mcd); // notice this is different from lbs_matrix


    std::vector<Eigen::MatrixXd> CSM_M;
    CSM_M.resize(d);

    int n = V.rows(); // number of mesh (domain) vertices
    Eigen::Matrix<int, Eigen::Dynamic, 1> span_n(n);
    for (int i = 0; i < n; i++) {
        span_n(i) = i;
    }


    Eigen::Matrix<int, Eigen::Dynamic, 1> span_mlbs_cols(Mcd.cols());
    for (int i = 0; i < Mcd.cols(); i++) {
        span_mlbs_cols(i) = i;
    }

    int n_groups = CSM.rows() / d;
    for (int i = 0; i < d; i++) {
        Eigen::MatrixXd M_i;
        igl::slice(Mcd, (span_n.array() + i * n).matrix().eval(), span_mlbs_cols, M_i);
        Eigen::MatrixXd M_i_dim;
        CSM_M[i].resize(n_groups * d, Mcd.cols());
        for (int j = 0; j < d; j++) {
            Eigen::SparseMatrix<double> CSMj;
            igl::slice(
                CSM,
                igl::colon<int>(j * n_groups,(j + 1) * n_groups - 1),
                igl::colon<int>(j * n,(j + 1) * n - 1),
                CSMj);
            assert(CSMj.rows() == n_groups); // remove in the end
            assert(CSMj.cols() == n); // remove in the end
            Eigen::MatrixXd CSMjM_i = CSMj * M_i;
            if (igl::is_sparse(CSMjM_i)) {
                // Convert to full
                Eigen::MatrixXd CSMjM_ifull(CSMjM_i);
                CSM_M[i].block(j * n_groups, 0, CSMjM_i.rows(), CSMjM_i.cols()) = CSMjM_ifull;
            }
            else {
                CSM_M[i].block(j * n_groups, 0, CSMjM_i.rows(), CSMjM_i.cols()) = CSMjM_i;
            }
        }
    }


    typedef Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> MatrixXS;
    
    std::vector<MatrixXS> CSM_M_SSCALAR;
    CSM_M_SSCALAR.resize(d);
    for (int i = 0; i < d; i++) {
        CSM_M_SSCALAR[i] = CSM_M[i].cast<float>();
    }

    // condense CSM
    MatrixXS nCSM;
    int numRows = CSM_M_SSCALAR[0].rows();
    int numCols = (d + 1) * m;
    nCSM.resize(numRows, numCols);

    for (int r = 0; r < numRows; r++) {
        for (int coord = 0; coord < d + 1; coord++) {
            for (int b = 0; b < m; b++) {
                // this is just a test if we really have a multiple of 3x3 identity
                Eigen::Matrix3f blok;
                for (int v = 0; v < 3; v++)
                {
                    for (int w = 0; w < 3; w++) {
                        blok(v, w) = CSM_M_SSCALAR[v](r, coord * (m * d) + b + w * m);
                    }          
                }

                nCSM(r, coord * m + b) = blok(0, 0);
            }
        }
    }



    std::cout << "T Rows: " << T.rows() << std::endl;
    std::cout << "T Cols: " << T.cols() << std::endl;


    std::cout << "nCSM Rows: " << nCSM.rows() << std::endl;
    std::cout << "nCSM Cols: " << nCSM.cols() << std::endl;



    // Overhang
    double tau = std::sin(config.alpha_max);

    // Retrieve parents for forward kinematics
    Eigen::MatrixXi P;
    igl::directed_edge_parents(BE, P);

    // Initialize initial guess `X` and bounds `LB`, `UB`

    bool is3d = !(V.cols() == 3 && V.col(2).sum() == 0.);
    int dim = d * m;
    Eigen::RowVectorXf X(dim), LB(dim), UB(dim);

    // find joint locations
    std::vector<Eigen::Vector3d> dT(BE.rows());
    for (int i = 0; i < BE.rows(); ++i) {
        dT[i] = C.row(BE.coeff(P(i) == -1 ? 0 : P(i), 1) - 1).cast<double>().transpose();
    }

    int k = 0;

    for (int j = 0; j < m; ++j) {

        k = d * j;

        // rotation 
        X(k)   = 0;
        X(k + 1) = 0;
        X(k + 2) = 0;

        if (is3d) {
            LB(k)     = -config.rotation_angle;
            LB(k + 1) = -config.rotation_angle;
            LB(k + 2) = -config.rotation_angle;
            UB(k)     =  config.rotation_angle;
            UB(k + 1) =  config.rotation_angle;
            UB(k + 2) =  config.rotation_angle;
        } else {
            // 2D rotation on {x,y}-plane amounts to 
            //      fixing rotation around {x,y}-axis, and allow rotation around z-axis
            LB(k)     = 0;
            LB(k + 1) = 0;
            LB(k + 2) = -config.rotation_angle;
            UB(k)     = 0;
            UB(k + 1) = 0;
            UB(k + 2) =  config.rotation_angle;
        }
    }

    
    // Energy function

    int iter = 0;
    const std::function<float(Eigen::RowVectorXf&)> f =
       [&n_groups,
        &iter, &config, &V, &nCSM, &G,
        &Cd, &BE, &P,                               // forward kinematics
        &T, &F, &U,                                 // mesh
        &M, &L, &K,                                 // arap
        &tau, &is3d                                 // overhang
    ](Eigen::RowVectorXf & X) -> float {

        MTR_SCOPE_FUNC();

        unzip(X, Cd, BE, P, T);
        U = M * T;


        double E_arap, E_overhang, E_intersect;

        std::vector<int> unsafe;
        // E_overhang = 0;
        E_overhang = overhang_energy(U, F, config.dp, tau, is3d?3:2, unsafe);


        // ARAP_ENERGY
        E_arap = 0;

        MatrixXS C = nCSM * T;

        MatrixXS R(C.rows(), C.cols());
        typedef Eigen::Matrix<float, 3, 3> Matrix3T;
        typedef Eigen::Matrix<float, 3, 1> Vector3T;
        Matrix3T Ck, Rk;
        for (int k = 0; k < n_groups; k++) {
            Ck = C.block(3 * k, 0, 3, 3);
            igl::polar_svd3x3(Ck, Rk);
            R.block(3 * k, 0, 3, 3) = Rk;
        }

        MTR_BEGIN("reduce_support", "arap_energy");

        int a, b; // two vertices on an edge
        double coeff;
        Matrix3T R_a;
        Vector3T new_vec, old_vec, old_vec_T, trans_old_vec, diff_vec;
        for (int i = 0; i < F.rows(); i++) {
            for (int j = 0; j < 3; j++) {
                a = F(i, j % 3);
                b = F(i, (j + 1) % 3);
                R_a = R.block(3 * G(a), 0, 3, 3);
                new_vec = U.row(a) - U.row(b);
                old_vec = V.row(a) - V.row(b);
                old_vec_T = old_vec.transpose();
                coeff = L.coeff(a, b);
                trans_old_vec = R_a * old_vec_T;
                diff_vec = new_vec - trans_old_vec;
                double diff = coeff * diff_vec.norm() * diff_vec.norm() * 1.0 / 6;
                E_arap += diff;
            }
        }

        MTR_END("reduce_support", "arap_energy");


        // Overlapping Energy
        E_intersect = 0;

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