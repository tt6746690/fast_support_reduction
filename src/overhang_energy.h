#ifndef __OVERHANG_ENERGY_H__
#define __OVERHANG_ENERGY_H__
#include <Eigen/Core>
#include <Eigen/Sparse>

#include "minitrace.h"

#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>

// Computes overhang energy
//      
//      E(V) = \int_{∂M} min(nˆ(x) · dp + τ, 0)dx
//
// Inputs:
//      V,  #V by dim
//      F,  #F by dim
//      bnd,            ordered list of boundary vertices of longest boundary loop
//                          outputs of igl::boundary_loop(F, bnd);
//      dp,             printing direction
//      tau,            self-supporting coefficient, τ = sin(α_max)
//      dim,            dimension {2, 3}
// Outputs:
//      U,              unsafe triangle for both {2, 3} dimension, i.e. indices into F
//      overhang energy E(V)

template <
    typename DerivedV,
    typename DerivedF,
    typename DerivedL>
double overhang_energy(
    const Eigen::MatrixBase<DerivedV>& V,
    const Eigen::MatrixBase<DerivedF>& F,
    const Eigen::PlainObjectBase<DerivedL>& bnd,
    const Eigen::RowVector3f& dp,
    double tau,
    int dim,
    std::vector<int>& U)
{
    MTR_SCOPE_FUNC();
    typedef typename DerivedV::Scalar ScalarV;
    typedef Eigen::Matrix<ScalarV, 3, 1> RowVector3VT;

    RowVector3VT dpn;
    dpn = dp.cast<ScalarV>().normalized();

    double e;
    double energy = 0;

    // 2D case

    if (dim == 2) {

        int bnd_size = bnd.size();
        RowVector3VT i, j, ele, n;
        std::vector<int> unsafe;

        for (int b = 0; b < bnd_size; ++b) {
            i = V.row(bnd(b));
            j = V.row(bnd((b+1)%bnd_size));
            ele = (i - j).normalized();
            n(0) = -ele(1);
            n(1) =  ele(0);
            n(2) =  ele(2);
            e = n.dot(dpn) + tau;
#ifdef VISUALIZE
            if (e < 0) {
                unsafe.push_back(bnd(b));
                unsafe.push_back(bnd((b+1)%bnd_size));
            }
#endif
            e = std::pow(std::min(e, 0.), 2.0);
            energy += e;
        }

    } else {
        printf("overhang energy for 3D not supported!\n");
    }

    return energy;
}

#endif