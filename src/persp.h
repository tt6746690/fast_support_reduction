#pragma once

#include <Eigen/Core>

// Implementation of the deprecated glOrtho function.
//
// Inputs:
//   left  coordinate of left vertical clipping plane
//   right  coordinate of right vertical clipping plane
//   bottom  coordinate of bottom vertical clipping plane
//   top  coordinate of top vertical clipping plane
//   nearVal  distance to near plane
//   farVal  distance to far plane
// Outputs:
//   P  4x4 perspective matrix
template <typename DerivedP>
void persp(
    const typename DerivedP::Scalar left,
    const typename DerivedP::Scalar right,
    const typename DerivedP::Scalar bottom,
    const typename DerivedP::Scalar top,
    const typename DerivedP::Scalar near,
    const typename DerivedP::Scalar far,
    Eigen::PlainObjectBase<DerivedP>& P)
{
    P.setConstant(4,4,0.);
    P(0,0) = (2.0 * near) / (right - left);
    P(1,1) = (2.0 * near) / (top - bottom);
    P(0,2) = (right + left) / (right - left);
    P(1,2) = (top + bottom) / (top - bottom);
    P(2,2) = -(far + near) / (far - near);
    P(3,2) = -1.0;
    P(2,3) = -(2.0 * far * near) / (far - near);
}
