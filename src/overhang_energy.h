#pragma once
#include <Eigen/Core>
#include <Eigen/Sparse>

#include <igl/per_face_normals.h>
#include <igl/doublearea.h>

#include "minitrace.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>

// Computes overhang energy for 2d and 3d cases
//
// Inputs:
//      V,  #V by dim
//      bnd,            ordered list of boundary vertices of longest boundary loop
//                          outputs of igl::boundary_loop(F, bnd);
//      dp,             printing direction (normalized)
//      tau,            self-supporting coefficient, τ = sin(α_max)
// Outputs:
//      unsafe, _ by 2  unsafe edges for both {2, 3} dimension, i.e. indices into F
//      overhang energy E(V)
template <
    typename DerivedV,
    typename DerivedL>
double overhang_energy_2d(
    const Eigen::MatrixBase<DerivedV>& V,
    const Eigen::PlainObjectBase<DerivedL>& bnd,
    const Eigen::RowVector3f& dp,
    double tau,
    Eigen::MatrixXi& unsafe)
{
    MTR_SCOPE_FUNC();
    typedef typename DerivedV::Scalar ScalarV;
    typedef Eigen::Matrix<ScalarV, 3, 1> RowVector3VT;

    double e;
    double energy = 0;

#ifdef VISUALIZE
    int k = 0;  // number of unsafe edges
    unsafe = Eigen::MatrixXi::Zero(bnd.rows()+1, 2);
#endif

    int bnd_size = bnd.size();
    RowVector3VT i, j, ele, n;

    for (int b = 0; b < bnd_size; ++b) {
        i = V.row(bnd(b));
        j = V.row(bnd((b+1)%bnd_size));
        ele = (i - j).normalized();
        n(0) = -ele(1);
        n(1) =  ele(0);
        n(2) =  ele(2);
        e = n.dot(dp) + tau;
#ifdef VISUALIZE
        if (e < 0) {
            unsafe.row(k) << bnd(b), bnd((b+1)%bnd_size);
            k += 1;
        }
#endif
        e = std::pow(std::min(e, 0.), 2.0);
        energy += e;
    }

#ifdef VISUALIZE
    unsafe.conservativeResize(k, unsafe.cols());
#endif

    return energy;
}


// Computes per-face centroid coordinate for triangle mesh in 3d
//
// Inputs:
//      V,  #V by dim
//      F,  #F by dim   surface meshes
// Outputs:
//      C,  #F by dim   per centroid coordinate
template <
    typename DerivedV,
    typename DerivedF,
    typename DerivedC>
void per_face_centroid(
    const Eigen::MatrixBase<DerivedV>& V,
    const Eigen::MatrixBase<DerivedF>& F,
    Eigen::MatrixBase<DerivedC>& C)
{
    for (int i = 0; i < F.rows(); ++i) {
        C.row(i) = (V.row(F(i, 0)) + V.row(F(i, 1)) + V.row(F(i, 2))) / 3;
    }
}

// Note V should be in first quadrant
// 
// Inputs:
//      V,  #V by dim
//      F,  #F by dim   surface meshes
//      dp,             printing direction (normalized)
//      tau,            self-supporting coefficient, τ = sin(α_max)
// Outputs:
//      overhang energy E(V)
template <
    typename DerivedV,
    typename DerivedF>
double overhang_energy_3d(
    const Eigen::MatrixBase<DerivedV>& V,
    const Eigen::MatrixBase<DerivedF>& F,
    const Eigen::RowVector3f& dp,
    double tau)
{
    MTR_SCOPE_FUNC();
    typedef typename DerivedV::Scalar ScalarV;
    typedef Eigen::Matrix<ScalarV, Eigen::Dynamic, 1> VectorXVT;
    typedef Eigen::Matrix<ScalarV, Eigen::Dynamic, Eigen::Dynamic> MatrixXVT;

    double negtau = -tau;
    double e;
    double energy = 0;

    MatrixXVT N;
    igl::per_face_normals(V, F, N);

    VectorXVT A;
    igl::doublearea(V, F, A);

    // height of rectangular prism, by projecting 
    //      centroid of triangle to printing direction
    VectorXVT h(F.rows());
    for (int i = 0; i < F.rows(); ++i) {
        h(i) = (V.row(F(i, 0)) + V.row(F(i, 1)) + V.row(F(i, 2))).dot(dp);
    }

    for (int i = 0; i < N.rows(); ++i) {
        e = N.row(i).dot(dp);
        if (e < negtau) {
            e = A(i) * e * h(i);
            energy += e;
        }
    }

    return std::abs(energy);
}



// Find risky edges for 3d printing
// 
// Inputs:
//      V,  #V by dim
//      F,  #F by dim   surface meshes
//      dp,             printing direction (normalized)
//      tau,            self-supporting coefficient, τ = sin(α_max)
// Outputs:
//      unsafe, _ by 2  unsafe edges for both {2, 3} dimension, i.e. indices into F
template <
    typename DerivedV,
    typename DerivedF,
    typename DerivedDP>
void overhang_energy_risky(
    const Eigen::MatrixBase<DerivedV>& V,
    const Eigen::MatrixBase<DerivedF>& F,
    const Eigen::MatrixBase<DerivedDP>& dp,
    double tau,
    Eigen::MatrixXi& unsafe)
{
    typedef typename DerivedV::Scalar ScalarV;
    typedef Eigen::Matrix<ScalarV, 3, 1> RowVector3VT;
    typedef Eigen::Matrix<ScalarV, Eigen::Dynamic, Eigen::Dynamic> MatrixXVT;

    RowVector3VT dpt = dp.template cast<ScalarV>();

    MatrixXVT N;
    igl::per_face_normals(V, F, N);

    double e;
    int k = 0;  // number of unsafe edges
    Eigen::RowVector3i t;
    unsafe = Eigen::MatrixXi::Zero(F.rows()*3, 2);

    for (int i = 0; i < N.rows(); ++i) {
        e = N.row(i).dot(dpt) + tau;
        if (e < 0) {
            t = F.row(i);
            unsafe.row(k)   << t(0), t(1);
            unsafe.row(k+1) << t(1), t(2);
            unsafe.row(k+2) << t(2), t(0);
            k += 3;
        }
    }
    unsafe.conservativeResize(k, unsafe.cols());
}
