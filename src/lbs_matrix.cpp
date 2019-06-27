#include "lbs_matrix.h"

#include <stan/math.hpp>

using namespace Eigen;

void lbs_matrix(
  const Eigen::MatrixXd & V, 
  const Eigen::MatrixXd & W,
  Matrix<stan::math::var,Dynamic,Dynamic> & M)
{
  // Number of dimensions
  const int dim = V.cols();
  // Number of model points
  const int n = V.rows();
  // Number of skinning transformations/weights
  const int m = W.cols();

  // Assumes that first n rows of weights correspond to V
  assert(W.rows() >= n);

  M.resize(n,(dim+1)*m);
  for(int j = 0;j<m;j++)
  {
    Matrix<stan::math::var,Dynamic,1> Wj = W.block(0,j,V.rows(),1);
    for(int i = 0;i<(dim+1);i++)
    {
      if(i<dim)
      {
        M.col(i + j*(dim+1)) = 
          Wj.cwiseProduct(V.col(i));
      }else
      {
        M.col(i + j*(dim+1)).array() = W.block(0,j,V.rows(),1).array();
      }
    }
  }
}