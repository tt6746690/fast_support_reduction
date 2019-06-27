#include "forward_kinematics.h"
#include <functional>
#include <iostream>

typedef
  std::vector<Eigen::Quaternion<stan::math::var>, Eigen::aligned_allocator<Eigen::Quaternion<stan::math::var>>>
  RotationList;

using namespace Eigen;
using namespace std;


void forward_kinematics(
  const Eigen::MatrixXd & C,
  const Eigen::MatrixXi & BE,
  const Eigen::VectorXi & P,
  const RotationList & dQ,
  const std::vector<Eigen::Vector3d> & dT,
  RotationList & vQ,
  std::vector<Matrix<stan::math::var,3,1>> & vT)
{
  const int m = BE.rows(); 
  assert(m == P.rows());
  assert(m == (int)dQ.size());
  assert(m == (int)dT.size());
  vector<bool> computed(m,false);
  vQ.resize(m);
  vT.resize(m);
  // Dynamic programming
  function<void (int) > fk_helper = [&] (int b)
  {
    if(!computed[b])
    {
      if(P(b) < 0)
      {
        // base case for roots
        vQ[b] = dQ[b];
        const Matrix<stan::math::var,3,1> r = C.row(BE(b,0)).transpose();
        vT[b] = r-dQ[b]*r + dT[b];
      }else
      {
        // Otherwise first compute parent's
        const int p = P(b);
        fk_helper(p);
        vQ[b] = vQ[p] * dQ[b];
        const Matrix<stan::math::var,3,1> r = C.row(BE(b,0)).transpose();
        vT[b] = vT[p] - vQ[b]*r + vQ[p]*(r + dT[b]);
      }
      computed[b] = true;
    }
  };
  for(int b = 0;b<m;b++)
  {
    fk_helper(b);
  }
}


void forward_kinematics(
  const Eigen::MatrixXd & C,
  const Eigen::MatrixXi & BE,
  const Eigen::VectorXi & P,
  const RotationList & dQ,
  const std::vector<Eigen::Vector3d> & dT,
  Matrix<stan::math::var, Dynamic, Dynamic> & T)
{
  RotationList vQ;
  vector<Matrix<stan::math::var,3,1>> vT;
  forward_kinematics(C,BE,P,dQ,dT,vQ,vT);
  const int dim = C.cols();
  T.resize(BE.rows()*(dim+1),dim);
  for(int e = 0;e<BE.rows();e++)
  {
    Transform<stan::math::var,3,Affine> a = Transform<stan::math::var,3,Affine>::Identity();
    a.translate(vT[e]);
    a.rotate(vQ[e]);
    T.block(e*(dim+1),0,dim+1,dim) =
      a.matrix().transpose().block(0,0,dim+1,dim);
  }
}


void forward_kinematics(
  const Eigen::MatrixXd & C,
  const Eigen::MatrixXi & BE,
  const Eigen::VectorXi & P,
  const RotationList & dQ,
  Matrix<stan::math::var,Dynamic,Dynamic> & T)
{
  std::vector<Eigen::Vector3d> dT(BE.rows(),Eigen::Vector3d(0,0,0));
  return forward_kinematics(C,BE,P,dQ,dT,T);
}