#ifndef __OVERHANG_ENERGY_H__
#define __OVERHANG_ENERGY_H__
#include <Eigen/Core>
#include <Eigen/Sparse>

// Computes overhang energy
//      
//      E(V) = \int_{∂M} min(nˆ(x) · dp + τ, 0)dx
//
// Inputs:
//      V,  #V by dim
//      F,  #F by dim
//      dp,             printing direction
//      tau,            self-supporting coefficient, τ = sin(α_max)
//      dim,            dimension {2, 3}
// Outputs:
//      overhang energy E(V)

template <
    typename DerivedV,
    typename DerivedF>
double overhang_energy(
    const Eigen::MatrixBase<DerivedV>& V,
    const Eigen::MatrixBase<DerivedF>& F,
    const Eigen::RowVector3f& dp,
    double tau,
    int dim,
    std::vector<int>& unsafe);

#endif