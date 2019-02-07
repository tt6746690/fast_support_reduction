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
    auto z = (eye - target).normalized();
    auto x = (up.cross(z)).normalized();
    auto y = (z.cross(x)).normalized();

    using Affine3S = Eigen::Transform<Scalar, 3, Eigen::Affine>;
    Affine3S view = Affine3S::Identity();
    view.matrix().block(0, 0, 3, 3) << x, y, z;
    view.matrix().block(3, 0, 1, 3) << -x.dot(eye), -y.dot(eye), -z.dot(eye);
    return view;
}