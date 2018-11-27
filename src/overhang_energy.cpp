#include "overhang_energy.h"
#include "minitrace.h"

#include <igl/slice.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/boundary_loop.h>
#include <igl/slice.h>

#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>

template <
    typename DerivedV,
    typename DerivedF>
double overhang_energy(
    const Eigen::MatrixBase<DerivedV>& V,
    const Eigen::MatrixBase<DerivedF>& F,
    const Eigen::RowVector3f& dp,
    double tau,
    int dim,
    std::vector<int>& unsafe)
{
    MTR_SCOPE_FUNC();
    typedef typename DerivedV::Scalar ScalarV;
    typedef Eigen::Matrix<ScalarV, 3, 1> RowVector3VT;
    typedef typename DerivedF::PlainObject PlainObjectF;

    RowVector3VT dpn;
    dpn = dp.cast<ScalarV>().normalized();

    double e;
    double energy = 0;

    // 2D case

    if (dim == 2) {

        Eigen::VectorXi bnd;
        igl::boundary_loop((PlainObjectF)F, bnd);

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
            if (e < 0) {
                unsafe.push_back(bnd(b));
                unsafe.push_back(bnd((b+1)%bnd_size));
            }
            e = std::pow(std::min(e, 0.), 2.0);
            energy += e;
        }

    } else {
        printf("overhang energy for 3D not supported!\n");
    }

    return energy;
}

// template specialization

template
double overhang_energy<Eigen::Matrix<float, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1> >(Eigen::MatrixBase<Eigen::Matrix<float, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, Eigen::Matrix<float, 1, 3, 1, 1, 3> const&, double, int, std::vector<int>&);


template
double overhang_energy<Eigen::Matrix<double, -1, -1, 0, -1, -1>, Eigen::Matrix<int, -1, -1, 0, -1, -1> >(Eigen::MatrixBase<Eigen::Matrix<double, -1, -1, 0, -1, -1> > const&, Eigen::MatrixBase<Eigen::Matrix<int, -1, -1, 0, -1, -1> > const&, Eigen::Matrix<float, 1, 3, 1, 1, 3> const&, double, int, std::__1::vector<int, std::__1::allocator<int> >&);