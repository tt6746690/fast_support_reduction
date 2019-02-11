#include <Eigen/Core>

//  Transforms `V` to be within [-1,1]^3
//  
//      V       #V x 3 vertex positions
template <
    typename DerivedV>
void normalized_device_coordinate(
    Eigen::MatrixBase<DerivedV>& V);


// implementation 

#include <cassert>

template <typename DerivedV>
void normalized_device_coordinate(
    Eigen::MatrixBase<DerivedV>& V)
{
    assert(V.cols() == 3 && "V.cols() is not 3\n");
    using RowVector3S = Eigen::Matrix<typename DerivedV::Scalar,3,1>;
    RowVector3S min, max, diff;
    min = V.colwise().minCoeff();
    max = V.colwise().maxCoeff();
    diff = max - min;
    for (int i = 0; i < V.rows(); ++i) {
        for (int j = 0; j < V.cols(); ++j) {
            V(i, j) = 2 * (V(i, j) - min(j)) / diff(j) - 1;
        }
    }
}
