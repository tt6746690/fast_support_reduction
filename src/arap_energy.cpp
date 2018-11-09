#include "arap_energy.h"

#include <igl/cotmatrix.h>
#include <vector>

void arap_precompute(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& F,
    const Eigen::MatrixXd& M,
    Eigen::MatrixXd& tL,
    Eigen::MatrixXd& tK)
{

    Eigen::MatrixXd Mt = M.transpose();

    // Compute \tilde{L}

    Eigen::SparseMatrix<double> L;
    igl::cotmatrix(V, F, L);
    tL = Mt * L * M;

    // Compute \tilde{K}
    
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
    Eigen::SparseMatrix<double> K;
    K.resize(nv, 3*nv);
    K.setFromTriplets(triplets.begin(), triplets.end());
    K = K / 6;
    tK = Mt * K;
}



float arap_compute(
    const Eigen::MatrixXd& T,
    const Eigen::MatrixXd& R,
    const Eigen::MatrixXd& tK,
    const Eigen::MatrixXd& tL) 
{
    Eigen::MatrixXd sum, Tt;
    Tt = T.transpose();
    sum = Tt*tL*T + Tt*tK*R;
    return sum.trace();
}