#include "arap_energy.h"

#include <igl/cotmatrix.h>
#include <igl/min_quad_with_fixed.h>
#include <igl/polar_svd3x3.h>
#include <vector>

void arap_precompute(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const Eigen::MatrixXd& M,
    const Eigen::VectorXi & b,
    Eigen::SparseMatrix<double>& L,
    Eigen::SparseMatrix<double>& K,
    igl::min_quad_with_fixed_data<double> & data)
{
    // Compute L
    igl::cotmatrix(V, F, L);

    // precompute U
    Eigen::SparseMatrix<double> Aeq;
    igl::min_quad_with_fixed_precompute(L, b, Aeq, false, data);

    // Compute K
    
    std::vector<Eigen::Triplet<double>> triplets;
    triplets.reserve(F.rows() * 3 * 3 * 3 * 2);

    int i, j, k;
    Eigen::RowVector3i f;
    Eigen::RowVector3d e;

    for (int a = 0; a < F.rows(); ++a) {
        f = F.row(a);
        // half edge in face `f`
        for (int b = 0; b < 3; ++b) {
            i = f(b%3);
            j = f((b+1)%3);
            k = f((b+2)%3);
            e = L.coeff(i, j) * (V.row(i) - V.row(j));
            // assign k' = {i,j,k} and d = {1,2,3} s.t.
            //      k_{i, 3k' + d} =  e^n_{ij}
            //      k_{j, 3k' + d} = -e^n_{ij}
            for (int d = 0; d < 3; ++d) {
                triplets.emplace_back(i, 3*i+d,  e(d));
                triplets.emplace_back(j, 3*i+d, -e(d));
                triplets.emplace_back(i, 3*j+d,  e(d));
                triplets.emplace_back(j, 3*j+d, -e(d));
                triplets.emplace_back(i, 3*k+d,  e(d));
                triplets.emplace_back(j, 3*k+d, -e(d));
            }
        }
    }

    int nv = V.rows();
    K.resize(nv, 3*nv);
    K.setFromTriplets(triplets.begin(), triplets.end());
    K = K / 6.0;
}

void arap_single_iteration(
    const igl::min_quad_with_fixed_data<double> & data,
    const Eigen::SparseMatrix<double> & K,
    const Eigen::MatrixXd & bc,
    Eigen::MatrixXd & U)
{
    Eigen::MatrixXd R(3 * U.rows(), 3);
    Eigen::MatrixXd C = U.transpose() * K;

    for (int i = 0; i < U.rows(); i++)
    {
        Eigen::Matrix3d Ri;
        Eigen::Matrix3d Ci = C.block(0, 3 * i, 3, 3).eval();
        Ci /= Ci.maxCoeff();
        igl::polar_svd3x3(Ci, Ri);
        R.block(3 * i, 0, 3, 3) = Ri.transpose();
    }

    Eigen::MatrixXd B = K * R;
    Eigen::VectorXd Beq; // empty constraint
    igl::min_quad_with_fixed_solve(data, B, bc, Beq, U);
}

double arap_compute(
    const Eigen::MatrixXd& T,
    const Eigen::MatrixXd& M,
    const Eigen::SparseMatrix<double>& L,
    const Eigen::SparseMatrix<double>& K)
{
    Eigen::MatrixXd sum, Tt, Mt, V, C, Ct;
    Mt = M.transpose();
    Tt = T.transpose();
    V = M * T;
    C = K.transpose() * V;
    Ct = C.transpose();

    // construct matrix R
    const int size = V.rows();
    Eigen::MatrixXd R(C.rows(), C.cols());
    Eigen::Matrix3d Ck, Rk;
    for (int k = 0; k < size; k++) {
        Ck = C.block(3 * k, 0, 3, 3);
        igl::polar_svd3x3(Ck, Rk);
        R.block(3 * k, 0, 3, 3) = Rk;
    }

    sum = 0.5 * Tt * Mt * L * M * T + Ct * R;
    double obj = sum.trace();
    return obj;
}