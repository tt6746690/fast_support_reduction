#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry> 

// construct view matrix at `eye` towards `target` where up is `up`
template <typename Scalar>
Eigen::Transform<Scalar, 3, Eigen::Affine> lookat_view(
        const Eigen::Matrix<Scalar, 3, 1> eye,
        const Eigen::Matrix<Scalar, 3, 1> target,
        const Eigen::Matrix<Scalar, 3, 1> up);

// implementation
template <typename Scalar>
Eigen::Transform<Scalar, 3, Eigen::Affine> lookat_view(
        const Eigen::Matrix<Scalar, 3, 1> eye,
        const Eigen::Matrix<Scalar, 3, 1> target,
        const Eigen::Matrix<Scalar, 3, 1> up)
{
    auto w = - (target - eye).normalized();
    auto u = (up.cross(w)).normalized();
    auto v = w.cross(u);

    using Affine3S = Eigen::Transform<Scalar, 3, Eigen::Affine>;
    Affine3S view = Affine3S::Identity();
    view.matrix().block(0, 0, 3, 4) << u, v, w, eye;
    return view;
}