#include "arap_energy.h"

#include <igl/cotmatrix.h>
#include <igl/min_quad_with_fixed.h>
#include <igl/polar_svd3x3.h>
#include <vector>

template<
    typename DerivedV,
    typename DerivedF,
    typename DerivedM,
    typename ScalarL,
    typename ScalarK>
void arap_precompute(
    const Eigen::MatrixBase<DerivedV>& V,
    const Eigen::MatrixBase<DerivedF>& F,
    const Eigen::MatrixBase<DerivedM>& M,
    Eigen::SparseMatrix<ScalarL>& L,
    Eigen::SparseMatrix<ScalarK>& K)
{
    typedef typename DerivedV::Scalar ScalarV;
    typedef typename DerivedF::Scalar ScalarF;
    typedef Eigen::Matrix<ScalarV, 3, 1> RowVector3VT;
    typedef Eigen::Matrix<ScalarF, 3, 1> RowVector3FT;

    // Compute L
    igl::cotmatrix(V, F, L);

    // Compute K
    
    std::vector<Eigen::Triplet<ScalarV>> triplets;
    triplets.reserve(F.rows() * 3 * 3 * 3 * 2);

    int i, j, k;
    RowVector3FT f;
    RowVector3VT e;

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
    K.resize(nv, 3 * nv);
    K.setFromTriplets(triplets.begin(), triplets.end());
    K = K / 6.0;
}

template<
    typename DerivedV,
    typename DerivedT,
    typename DerivedM,
    typename DerivedF,
    typename ScalarL,
    typename ScalarK>
double arap_energy(
    const Eigen::MatrixBase<DerivedV>& V,
    const Eigen::MatrixBase<DerivedT>& T,
    const Eigen::MatrixBase<DerivedM>& M,
    const Eigen::MatrixBase<DerivedF>& F,
    const Eigen::SparseMatrix<ScalarL>& L,
    const Eigen::SparseMatrix<ScalarK>& K)
{
    typedef typename DerivedT::Scalar ScalarT;
    typedef Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic> MatrixXT;
    typedef Eigen::Matrix<ScalarT, 3, 3> Matrix3T;
    typedef Eigen::Matrix<ScalarT, 3, 1> Vector3T;

    MatrixXT U, C;
    U = M * T;
    C = K.transpose() * U;

    // construct matrix R
    const int size = U.rows();
    MatrixXT R(C.rows(), C.cols());
    Matrix3T Ck, Rk;
    for (int k = 0; k < size; k++) {
        Ck = C.block(3 * k, 0, 3, 3);
        igl::polar_svd3x3(Ck, Rk);
        R.block(3 * k, 0, 3, 3) = Rk;
    }


    double obj = 0;
    int a, b;
    double coeff;
    Matrix3T R_a;
    Vector3T new_vec, old_vec, old_vec_T, trans_old_vec, diff_vec;
    for (int i = 0; i < F.rows(); i++) {
        for (int j = 0; j < 3; j++) {
            a = F(i, j % 3);
            b = F(i, (j + 1) % 3);
            R_a = R.block(3 * a, 0, 3, 3);
            new_vec = U.row(a) - U.row(b);
            old_vec = V.row(a) - V.row(b);
            old_vec_T = old_vec.transpose();
            coeff = L.coeff(a, b);
            trans_old_vec = R_a * old_vec_T;
            diff_vec = new_vec - trans_old_vec;
            double diff = coeff * diff_vec.norm() * diff_vec.norm() * 1.0 / 6;
            obj += diff;
        }
    }


    return obj;
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



// template specialization 

template
void arap_precompute<Eigen::Matrix<float, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1>, Eigen::Matrix<float, -1, -1, 0, -1, -1>, float, float>(Eigen::MatrixBase<Eigen::Matrix<float, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<float, -1, -1, 0, -1, -1> > const&, Eigen::SparseMatrix<float, 0, int>&, Eigen::SparseMatrix<float, 0, int>&);

template
double arap_energy<Eigen::Matrix<float, -1, -1, 0, -1, -1>, Eigen::Matrix<float, -1, -1, 0, -1, -1>, Eigen::Matrix<float, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1>, float, float>(Eigen::MatrixBase<Eigen::Matrix<float, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<float, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<float, -1, -1, 0,-1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, Eigen::SparseMatrix<float, 0, int> const&, Eigen::SparseMatrix<float, 0, int> const&);



template
void arap_precompute<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1>, Eigen::Matrix<double, -1, -1, 0, -1, -1>, double, double>(Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::SparseMatrix<double, 0, int>&, Eigen::SparseMatrix<double, 0, int>&);
