#include "arap_energy.h"

#include <igl/cotmatrix.h>
#include <igl/polar_svd3x3.h>
#include <vector>

void arap_precompute(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const Eigen::MatrixXd& M,
    Eigen::SparseMatrix<double>& L,
    Eigen::SparseMatrix<double>& K)
{
    // Compute L
    igl::cotmatrix(V, F, L);

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