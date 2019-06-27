#include <stan/math.hpp>

using namespace Eigen;

void lbs_matrix(
    const Eigen::MatrixXd & V, 
    const Eigen::MatrixXd & W,
    Matrix<stan::math::var,Dynamic,Dynamic> & M);
