#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>

#include <stan/math.hpp>

using namespace Eigen;
using namespace std;

void compute_jacobian(
Matrix<stan::math::var,Dynamic,1> & V,
Matrix<stan::math::var,Dynamic,1> & fV,
Matrix<double,Dynamic,Dynamic> & J) {

    // J.resize(fV.size(), V.size());
    // for (int i = 0; i < fV.size(); ++i) {
    //     if (i > 0) stan::math::set_zero_all_adjoints();
    //     fV(i).grad();
    //     for (int j = 0; j < V.size(); ++j) {
    //         J(i,j) = V(j).adj();
    //     }
    // }

    stan::math::start_nested();
    J.resize(fV.size(), V.size());
    for (int i = 0; i < fV.size(); ++i) {
        if (i > 0)
            stan::math::set_zero_all_adjoints_nested();
        stan::math::grad(fV(i).vi_);
        for (int k = 0; k < V.size(); ++k)
            J(i, k) = V(k).adj();
    }

}