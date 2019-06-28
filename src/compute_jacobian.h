#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>

#include <stan/math.hpp>

using namespace Eigen;
using namespace std;

void compute_jacobian(
Matrix<stan::math::var,Dynamic,1> & V,  // input
Matrix<stan::math::var,Dynamic,1> & fV, // output
Matrix<double,Dynamic,Dynamic> & J);