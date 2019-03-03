#include "element.h"

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>
#include <iostream>

using namespace Eigen;
using namespace std;

Element::Element(const Vector4i& ids)
{
    nodesIds = ids;
    B.resize(6, 12);
    B.setZero();
}

void Element::constructStiffnessMatrix(const MatrixXd& V, const MatrixXd& E, vector<Triplet<double>>& triplets)
{
    // construct C matrix for each tet
    MatrixXd C(4, 4);
    C.block(0, 0, 1, 4) = RowVector4d(1.0, 1.0, 1.0, 1.0);
    for (int i = 0; i < 4; i++) {
        C.block(1, i, 3, 1) = V.row(nodesIds(i)).transpose();
    }
    volume = C.determinant()*1.0/6;
    assert(volume > 0);
    C = C.inverse();

    // contruct B matrix for each tet
    for (int k = 0; k < 4; k++) {
        B(0, k*3) = C(k, 1);
        B(1, k*3+1) = C(k, 2);
        B(2, k*3+2) = C(k, 3);
        B(3, k*3) = C(k, 2);
        B(3, k*3+1) = C(k, 1);
        B(4, k*3+1) = C(k, 3);
        B(4, k*3+2) = C(k, 2);
        B(5, k*3) = C(k, 3);
        B(5, k*3+2) = C(k, 1);
    }

    // std::cout << "B: " << std::endl;
    // std::cout << B << std::endl;

    // construct element stiffness matrix Ki for each tet
    MatrixXd Ki(12, 12);
    Ki = B.transpose() * E * B * volume;

    for (int m = 0; m < 4; m++) {
        for (int n = 0; n < 4; n++) {
            Triplet trplt11(3 * nodesIds(m) + 0, 3 * nodesIds(n) + 0, Ki(3 * m + 0, 3 * n + 0));
            Triplet trplt12(3 * nodesIds(m) + 0, 3 * nodesIds(n) + 1, Ki(3 * m + 0, 3 * n + 1));
            Triplet trplt13(3 * nodesIds(m) + 0, 3 * nodesIds(n) + 2, Ki(3 * m + 0, 3 * n + 2));
            Triplet trplt21(3 * nodesIds(m) + 1, 3 * nodesIds(n) + 0, Ki(3 * m + 1, 3 * n + 0));
            Triplet trplt22(3 * nodesIds(m) + 1, 3 * nodesIds(n) + 1, Ki(3 * m + 1, 3 * n + 1));
            Triplet trplt23(3 * nodesIds(m) + 1, 3 * nodesIds(n) + 2, Ki(3 * m + 1, 3 * n + 2));
            Triplet trplt31(3 * nodesIds(m) + 2, 3 * nodesIds(n) + 0, Ki(3 * m + 2, 3 * n + 0));
            Triplet trplt32(3 * nodesIds(m) + 2, 3 * nodesIds(n) + 1, Ki(3 * m + 2, 3 * n + 1));
            Triplet trplt33(3 * nodesIds(m) + 2, 3 * nodesIds(n) + 2, Ki(3 * m + 2, 3 * n + 2));

            triplets.push_back(trplt11);
            triplets.push_back(trplt12);
            triplets.push_back(trplt13);
            triplets.push_back(trplt21);
            triplets.push_back(trplt22);
            triplets.push_back(trplt23);
            triplets.push_back(trplt31);
            triplets.push_back(trplt32);
            triplets.push_back(trplt33);
        }
    }

}