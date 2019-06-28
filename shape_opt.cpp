#include <iostream>
#include <stan/math.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <boost/math/tools/promotion.hpp>
#include <functional>

#include "compute_jacobian.h"
#include "forward_kinematics.h"
#include "lbs_matrix.h"

#include <igl/opengl/glfw/Viewer.h>
#include <igl/directed_edge_parents.h>
#include <igl/forward_kinematics.h>
#include <igl/lbs_matrix.h>
#include <igl/readOBJ.h>
#include <igl/readDMAT.h>
#include <igl/readTGF.h>

using namespace Eigen;
using namespace std;

typedef
  std::vector<Eigen::Quaternion<stan::math::var>, Eigen::aligned_allocator<Eigen::Quaternion<stan::math::var>>>
  RotationList;


int selected = 0;
MatrixXd V, W, C, T;
MatrixXi F, BE;

const Eigen::RowVector3d red(255./255.,0./255.,0./255.);
const double young = 1.45e5; // Young's modulus
const double mu = 0.45; // possion ratio
const double g = -9.8;


inline string getfilepath(const string& name, const string& ext) { 
    return "../data/" + name + "." + ext; 
};

void set_color(igl::opengl::glfw::Viewer &viewer)
{
  Eigen::MatrixXd C;
  igl::jet(W.col(selected).eval(),true,C);
  viewer.data().set_colors(C);
}

bool key_down(igl::opengl::glfw::Viewer &viewer, unsigned char key, int mods)
{
  switch(key)
  {
    case ' ':
      viewer.core.is_animating = !viewer.core.is_animating;
      break;
    case '.':
      selected++;
      selected = std::min(std::max(selected,0),(int)W.cols()-1);
      set_color(viewer);
      break;
    case ',':
      selected--;
      selected = std::min(std::max(selected,0),(int)W.cols()-1);
      set_color(viewer);
      break;
  }
  return true;
}



int main(int argc, char *argv[])
{
    igl::opengl::glfw::Viewer viewer;
    string filename = "cantilever";
    igl::readOBJ(getfilepath(filename, "obj"), V, F);
    igl::readDMAT(getfilepath(filename, "dmat"), W);
    igl::readTGF(getfilepath(filename, "tgf"), C, BE);

    Matrix<stan::math::var,Dynamic,Dynamic> U(V.rows(),V.cols());
    Matrix<stan::math::var,Dynamic,Dynamic> M;

    const int dim = 2;
    const int num_V = V.rows();

    int m = W.cols(); // number of bones
    int d = V.cols();

    U = V;
    T.resize((d+1)*m, d);

    // LBS
    lbs_matrix(V, W, M);
    // Retrieve parents for forward kinematics
    Eigen::MatrixXi P;
    igl::directed_edge_parents(BE, P);


    Matrix<stan::math::var,Dynamic,1> angles(4);
    angles.setZero();


    // init X
    Matrix<stan::math::var,1,Dynamic> X(d*m);
    X.setZero();
    for (int j = 0; j < m; ++j) {
        int k = d*j;
        X(k+2) = angles(j); // 2d
    }

    // Construct list of relative rotations in terms of quaternion
    // from Euler's angle
    RotationList dQ;
    Matrix<stan::math::var,1,3> th;
    for (int j = 0; j < BE.rows(); ++j) {
        th = X.segment(3*j, 3);
        dQ.emplace_back(
            Eigen::AngleAxis<stan::math::var>(th(0), Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxis<stan::math::var>(th(1), Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxis<stan::math::var>(th(2), Eigen::Vector3d::UnitZ())
        );
    }


    // Forward kinematics
    Matrix<stan::math::var,Dynamic,Dynamic> T;
    forward_kinematics(C, BE, P, dQ, T);
    U = M*T;

    Matrix<stan::math::var,Dynamic,Dynamic> U_2;
    U_2.resize(num_V, dim);
    U_2 << U.col(0), U.col(1);


    // Finite Element
    bool fem = true;
    if (fem) {
        // dirichlet boundary condition
        vector<int> fixedVertices;
        double tolerance = (V.col(0).maxCoeff() - V.col(0).minCoeff()) * 0.005;
        double min_Y = V.col(0).minCoeff();
        for (int i = 0; i < V.rows(); i++) {
            if (abs(V(i, 0)-min_Y) < tolerance) {
                fixedVertices.push_back(i);
            }
        }


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

        Matrix<stan::math::var,3,3> C_;
        Matrix<stan::math::var,3,3> IC;
        Matrix<stan::math::var,3,6> B;
        B.setZero();
        Matrix<stan::math::var,6,6> Ke;
        stan::math::var tri_area;

        Matrix<stan::math::var, Dynamic, Dynamic> Bs(3*F.rows(),6);

        for (int i = 0; i < F.rows(); i++) {

            auto ele_i = F.row(i);
            auto v0 = U_2.row(ele_i(0));
            auto v1 = U_2.row(ele_i(1));
            auto v2 = U_2.row(ele_i(2));

            C_ << 1, v0,
                1, v1,
                1, v2;

            IC = C_.inverse();
            tri_area = C_.determinant()/2;

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
        Matrix<stan::math::var, Dynamic, 1> u(dim*num_V);
        u = solver.solve(f); // displacements

        Matrix<double, Dynamic, 1> u_d(dim*num_V);
        for (int i = 0; i < dim*num_V; i++) {
            u_d(i) = u(i).val();
        }

        cout << u_d.bottomRows(10) << endl;

        Matrix<stan::math::var,Dynamic,1> t1(dim*num_V);
        t1 = K*u_d;

        Matrix<double,Dynamic,Dynamic> J1;
        compute_jacobian(angles, t1, J1); // slow

        cout << J1.bottomRows(20) << endl;

        cout << "-----------" << endl;


        Matrix<double,Dynamic,Dynamic> J2;
        compute_jacobian(angles, f, J2); // slow

        cout << J2.bottomRows(20) << endl;

    };

    viewer.data().set_mesh(V, F);
    set_color(viewer);
    viewer.data().set_edges(C, BE, red);
    viewer.data().add_points(C, red);
    viewer.data().show_lines = false;
    viewer.data().line_width = 10;
    viewer.callback_key_down = &key_down;
    viewer.launch();

    return 0;
}

// DEBUGGING
// for (auto i = fixedVertices.begin(); i != fixedVertices.end(); ++i) {
//     cout << *i << endl;
// }


// Matrix<stan::math::var, Dynamic, Dynamic> U(num_V, 3);
// for (int i = 0; i < num_V; i++) {
//     U(i, 0) = u(2*i,0);
//     U(i, 1) = u(2*i+1,0);
//     U(i, 2) = 0;
// }
// U = V+U;
// V = U.cast<double>().eval();


// cout << u.bottomRows(10) << endl;


// int counter = 0;
// for (int i = 0; i < t1.size(); i++) {
//     for (int j = 0; j < angles.size(); j++) {
//         if (J1(i,j) != 0) {
//             counter++;
//         } 
//     }
// }
// cout << J1 << endl;
