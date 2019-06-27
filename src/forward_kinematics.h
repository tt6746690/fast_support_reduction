#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>

#include <stan/math.hpp>

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
std::vector<Matrix<stan::math::var,3,1>> & vT);



// Outputs:
//   T  #BE*(dim+1) by dim stack of transposed transformation matrices
void forward_kinematics(
const Eigen::MatrixXd & C,
const Eigen::MatrixXi & BE,
const Eigen::VectorXi & P,
const RotationList & dQ,
const std::vector<Eigen::Vector3d> & dT,
Matrix<stan::math::var, Dynamic, Dynamic> & T);



void forward_kinematics(
const Eigen::MatrixXd & C,
const Eigen::MatrixXi & BE,
const Eigen::VectorXi & P,
const RotationList & dQ,
Matrix<stan::math::var, Dynamic, Dynamic> & T);