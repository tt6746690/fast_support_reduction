#ifndef __OVERHANG_ENERGY_H__
#define __OVERHANG_ENERGY_H__
#include <Eigen/Core>
#include <Eigen/Sparse>

// Computes overhang energy
//      
//      E(V) = \int_{∂M} min(nˆ(x) · dp − τ, 0)dx
//
// Inputs:
//      V,  #V by dim
//      F,  #F by dim
//      dp,             printing direction
//      tau,            self-supporting coefficient, τ = sin(α_max)
// Outputs:
//      overhang energy E(V)
double overhang_energy(
    const Eigen::MatrixXd V,
    const Eigen::MatrixXi F,
    const Eigen::RowVector3d dp,
    const double tau,
    bool display);


#endif