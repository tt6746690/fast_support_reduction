#include <iostream>
#include <stan/math.hpp>
#include <Eigen/Core>
#include <boost/math/tools/promotion.hpp>

#include <igl/opengl/glfw/Viewer.h>
#include <igl/readOBJ.h>

using namespace Eigen;
using namespace std;


inline string getfilepath(const string& name, const string& ext) { 
    return "../data/" + name + "." + ext; 
};


MatrixXd V, V_2;
MatrixXi F;

const double young = 1.45e5; // Young's modulus
const double mu = 0.45; // possion ratio
const double g = -9.8;

int main(int argc, char *argv[])
{

    igl::opengl::glfw::Viewer viewer;
    string filename = "cantilever_new";
    igl::readOBJ(getfilepath(filename, "obj"), V, F);

    const int dim = 2;
    const int num_V = V.rows();

    V_2.resize(num_V, dim);
    V_2 << V.col(0), V.col(1);

    // dirichlet boundary condition
    vector<int> fixedVertices;
    double tolerance = (V.col(0).maxCoeff() - V.col(0).minCoeff()) * 0.005;
    double min_Y = V.col(0).minCoeff();
    for (int i = 0; i < V.rows(); i++) {
        if (abs(V(i, 0)-min_Y) < tolerance) {
            fixedVertices.push_back(i);
        }
    }

    // for (auto i = fixedVertices.begin(); i != fixedVertices.end(); ++i) {
    //     cout << *i << endl;
    // }

    // prepare K and f
    SparseMatrix<stan::math::var> K;
    K.resize(dim*num_V, dim*num_V);
    Matrix<stan::math::var, Dynamic, 1> f(dim*num_V);
    f.setZero();

    // construct the elasticity matrix D
    MatrixXd D(3, 3);
    D << 1, mu, 0,
        mu,  1, 0,
         0,  0, 0.5*(1-mu);
    D *= young/(1-mu*mu);

    vector<Triplet<stan::math::var>> triplets;
    triplets.reserve(F.rows()*3*3*2*2);

    Matrix<stan::math::var, 3, 3> C;
    Matrix<stan::math::var, 3, 3> IC;
    Matrix<stan::math::var, 3, 6> B;
    B.setZero();
    Matrix<stan::math::var, 6, 6> Ke;
    stan::math::var tri_area;

    Matrix<stan::math::var, Dynamic, Dynamic> Bs(3*F.rows(),6);

    for (int i = 0; i < F.rows(); i++) {

        auto ele_i = F.row(i);
        auto v0 = V_2.row(ele_i(0));
        auto v1 = V_2.row(ele_i(1));
        auto v2 = V_2.row(ele_i(2));

        C << 1, v0,
             1, v1,
             1, v2;

        IC = C.inverse();
        tri_area = C.determinant()/2;

        assert(tri_area != 0);

        for (int j = 0; j < 3; j++) {
            B(0, 2*j+0) = IC(1, j);
            B(0, 2*j+1) = 0;
            B(1, 2*j+0) = 0;
            B(1, 2*j+1) = IC(2, j);
            B(2, 2*j+0) = IC(2, j);
            B(2, 2*j+1) = IC(1, j);
        }

        Ke = B.transpose().eval()*D*B*tri_area;

        // assemble f
        for (int j = 0; j < 3; j++) {
            f(2*ele_i(j)+1, 0) += g*tri_area/3;
        }

        // assemble K
        for (int m = 0; m < 3; m++) {
            for (int n = 0; n < 3; n++) {
                Triplet trplt11(2*ele_i(m)+0, 2*ele_i(n)+0, Ke(2*m+0, 2*n+0));
                Triplet trplt12(2*ele_i(m)+0, 2*ele_i(n)+1, Ke(2*m+0, 2*n+1));
                Triplet trplt21(2*ele_i(m)+1, 2*ele_i(n)+0, Ke(2*m+1, 2*n+0));
                Triplet trplt22(2*ele_i(m)+1, 2*ele_i(n)+1, Ke(2*m+1, 2*n+1));

                triplets.push_back(trplt11);
                triplets.push_back(trplt12);
                triplets.push_back(trplt21);
                triplets.push_back(trplt22);
            }
        }

        Bs.block(3*i, 0, 3, 6) = B; // save B
        
    }

    K.setFromTriplets(triplets.begin(), triplets.end());

    // apply constraints
    for (int i = 0; i < fixedVertices.size(); i++) {
        int idx = fixedVertices[i];
        K.prune([&idx](int m, int n, stan::math::var) { 
            return m!=2*idx && n!=2*idx && m!=2*idx+1 && n!=2*idx+1; });
        K.coeffRef(2*idx, 2*idx) = 1;
        K.coeffRef(2*idx+1, 2*idx+1) = 1;
        f(2*idx) = 0;
        f(2*idx+1) = 0;
    }

    // solve for displacements at each vertex Kd = f
	SimplicialLDLT<SparseMatrix<stan::math::var>> solver(K);
    auto u = solver.solve(f); // displacements

    cout << u.bottomRows(10) << endl;

    viewer.data().set_mesh(V, F);
    viewer.launch();

    return 0;
}

// Matrix<stan::math::var, Dynamic, Dynamic> U(num_V, 3);
// for (int i = 0; i < num_V; i++) {
//     U(i, 0) = u(2*i,0);
//     U(i, 1) = u(2*i+1,0);
//     U(i, 2) = 0;
// }
// U = V+U;
// V = U.cast<double>().eval();