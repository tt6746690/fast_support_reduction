#ifndef __OVERHANG_ENERGY_H__
#define __OVERHANG_ENERGY_H__
#include <Eigen/Core>
#include <Eigen/Sparse>

#include <igl/per_face_normals.h>

#include "minitrace.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>

// Computes overhang energy for 2d and 3d cases
//      
//      E(V) = \int_{∂M} min(nˆ(x) · dp + τ, 0)dx
//
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


// Inputs:
//      V,  #V by dim
//      F,  #F by dim   surface meshes
//      dp,             printing direction (normalized)
//      tau,            self-supporting coefficient, τ = sin(α_max)
// Outputs:
//      unsafe, _ by 2  unsafe edges for both {2, 3} dimension, i.e. indices into F
//      overhang energy E(V)
template <
    typename DerivedV,
    typename DerivedF>
double overhang_energy_3d(
    const Eigen::MatrixBase<DerivedV>& V,
    const Eigen::MatrixBase<DerivedF>& F,
    const Eigen::RowVector3f& dp,
    double tau,
    Eigen::MatrixXi& unsafe)
{
    MTR_SCOPE_FUNC();
    typedef typename DerivedV::Scalar ScalarV;
    typedef Eigen::Matrix<ScalarV, Eigen::Dynamic, Eigen::Dynamic> MatrixXVT;

    double e;
    double energy = 0;

    MatrixXVT N;
    igl::per_face_normals(V, F, N);

#ifdef VISUALIZE
    int k = 0;  // number of unsafe edges
    Eigen::RowVector3i t;
    unsafe = Eigen::MatrixXi::Zero(F.rows()*3, 2);
#endif

    for (int i = 0; i < N.rows(); ++i) {
        e = N.row(i).dot(dp) + tau;

#ifdef VISUALIZE
        if (e < 0) {
            // std::cout << "theta (degrees): " << ( (1. - std::acos(N.row(i).normalized().dot(dp)) / M_PI) * 180) << ";  tau: " << tau << '\n';
            t = F.row(i);
            unsafe.row(k)   << t(0), t(1);
            unsafe.row(k+1) << t(1), t(2);
            unsafe.row(k+2) << t(2), t(0);
            k += 3;
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


#endif