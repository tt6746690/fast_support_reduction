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
#include <igl/deform_skeleton.h>
#include <igl/readOBJ.h>
#include <igl/readDMAT.h>
#include <igl/readTGF.h>

#include <boost/math/tools/minima.hpp>

#include "minitrace.h"


using namespace Eigen;
using namespace std;

typedef
  std::vector<Eigen::Quaternion<stan::math::var>, Eigen::aligned_allocator<Eigen::Quaternion<stan::math::var>>>
  RotationList;


int selected = 0;
MatrixXd V, W, C, CT, T;
MatrixXi F, BE, BET;

const Eigen::RowVector3d blue(0.3,0.4,1.0);
const Eigen::RowVector3d orange(0.8,0.5,0.2);
const Eigen::RowVector3d red(255./255.,0./255.,0./255.);
const double young = 1.45e5; // Young's modulus
const double mu = 0.45; // possion ratio
const double g = -98;


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




struct objfunc
{
    objfunc(Matrix<double, Dynamic, 1> & x, Matrix<double, Dynamic, 1> & dx, int m, int d, MatrixXi & BE, MatrixXd & C, MatrixXi & P)
             : x(x), dx(dx), m(m), d(d), BE(BE), C(C), P(P){}
    double operator()(double const& t) {

        Vector3d x_new = x-t*dx;
        Matrix<double,1,Dynamic> X(d*m);
        X.setZero();
        // update X
        for (int j = 1; j < m; ++j) {
            int k = d*j;
            X(k+2) = x_new(j-1); // 2d
        }

        RotationList dQ;
        Matrix<double,1,3> th;
        for (int j = 0; j < BE.rows(); ++j) {
            th = X.segment(3*j, 3);
            dQ.emplace_back(
                Eigen::AngleAxis<double>(th(0), Eigen::Vector3d::UnitX()) *
                Eigen::AngleAxis<double>(th(1), Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxis<double>(th(2), Eigen::Vector3d::UnitZ())
            );
        }

        // Matrix<double,Dynamic,Dynamic> T;
        // igl::forward_kinematics(C, BE, P, dQ, T);


        double result = (x+t*dx).transpose().eval()*(x+t*dx);
        // double result = (t+3)*(t-2);
        return result;
    }

    private:
        Matrix<double, Dynamic, 1> & x;
        Matrix<double, Dynamic, 1> & dx;
        int m, d;
        MatrixXi & BE;
        MatrixXd & C;
        MatrixXi & P;

};




int main(int argc, char *argv[])
{

    mtr_init("trace.json");

    igl::opengl::glfw::Viewer viewer;
    string filename = "cantilever_S";
    igl::readOBJ(getfilepath(filename, "obj"), V, F);
    igl::readDMAT(getfilepath(filename, "dmat"), W);
    igl::readTGF(getfilepath(filename, "tgf"), C, BE);


    Matrix<stan::math::var,Dynamic,Dynamic> U; U.resizeLike(V);
    Matrix<stan::math::var,Dynamic,Dynamic> M;
    Matrix<double,Dynamic,Dynamic> M_d;

    const int dim = V.cols()-1;
    const int num_V = V.rows();

    int m = W.cols(); // number of bones
    int d = V.cols();

    U = V;
    T.resize((d+1)*m, d);

    // LBS
    igl::lbs_matrix(V, W, M_d);

    lbs_matrix(V, W, M);
    // Retrieve parents for forward kinematics
    Eigen::MatrixXi P;
    igl::directed_edge_parents(BE, P);


    // dirichlet boundary condition
    vector<int> fixedVertices;
    double tolerance = (V.col(0).maxCoeff() - V.col(0).minCoeff()) * 0.005;
    double min_Y = V.col(0).minCoeff();
    for (int i = 0; i < V.rows(); i++) {
        if (abs(V(i, 0)-min_Y) < tolerance) {
            fixedVertices.push_back(i);
        }
    }


    // construct the elasticity matrix D
    MatrixXd D(3, 3);
    D << 1, mu, 0,
        mu,  1, 0,
        0,  0, 0.5*(1-mu);
    D *= young/(1-mu*mu);


    Matrix<double, Dynamic, 1> u_d(dim*num_V);
    SparseMatrix<double> K_d;

    auto f = [&] (Matrix<stan::math::var,Dynamic,1> & angles_var) {

        // init X
        Matrix<stan::math::var,1,Dynamic> X(d*m);
        X.setZero();
        // update X
        for (int j = 1; j < m; ++j) {
            int k = d*j;
            X(k+2) = angles_var(j-1); // 2d
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


        // prepare K and f
        SparseMatrix<stan::math::var> K;
        K.resize(dim*num_V, dim*num_V);
        Matrix<stan::math::var, Dynamic, 1> f(dim*num_V);
        f.setZero();


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

        for (int i = 0; i < dim*num_V; i++) {
            u_d(i) = u(i).val();
        }

        Matrix<stan::math::var,Dynamic,1> t(dim*num_V);
        t = f-K*u_d;

        K_d.resize(K.rows(),K.cols());
        for (int i = 0; i < K.outerSize(); ++i) {
            // Iterate over inside
            for (SparseMatrix<stan::math::var>::InnerIterator it (K,i); it; ++it) {
                // it.row(),  it.col(), it.value()
                K_d.coeffRef(it.row(),it.col()) = it.value().val();
            }
        }

        return t;
    };



    // opt1
    // Matrix<double, Dynamic, Dynamic> tar(dim*num_V,m-1);
    // Matrix<double, Dynamic, Dynamic> dx(m-1,1);
    // Eigen::MatrixXd J;
    // Eigen::VectorXd fx;

    // // design variables: euler angle
    // Matrix<double,Dynamic,1> x(m-1);
    // x.setZero();
    // for (int i = 0; i < m-1; i++) {
    //     x(i) = 0.3;
    // }

    // // gradient descent
    // double tol = 1e-5;
    // double diff = 100;

    // double step_size = 0.01; // fixed step size

    // while (diff > tol) {

    //     MTR_BEGIN("C++", "total");

    //     // stan::math::jacobian(f, x, fx, J);
    //     stan::math::start_nested();
    //     try {
    //         Matrix<stan::math::var, Dynamic, 1> x_var(x.size());
    //         for (int k = 0; k < x.size(); ++k)
    //             x_var(k) = x(k);
    //         Matrix<stan::math::var, Dynamic, 1> fx_var = f(x_var);
    //         fx.resize(fx_var.size());
    //         for (int i = 0; i < fx_var.size(); ++i)
    //             fx(i) = fx_var(i).val();
    //         J.resize(fx_var.size(), x.size());

    //         MTR_BEGIN("C++", "jacobian");

    //         for (int i = 0; i < fx_var.size(); ++i) {
    //             if (i > 0)
    //                 stan::math::set_zero_all_adjoints_nested();
    //             stan::math::grad(fx_var(i).vi_);
    //             for (int k = 0; k < x.size(); ++k)
    //                 J(i, k) = x_var(k).adj();
    //         }

    //         MTR_END("C++", "jacobian");  

    //     } catch (const std::exception& e) {
    //         stan::math::recover_memory_nested();
    //         throw;
    //     }
    //     stan::math::recover_memory_nested();

    //     MTR_END("C++", "total");  

    //     SimplicialLDLT<SparseMatrix<double>> solver_d(K_d);
    //     tar = solver_d.solve(J);
    //     dx = 2*tar.transpose().eval()*u_d;

    //     x = x-step_size*dx;

    //     diff = dx.norm();

    //     cout << "x: " << x << endl; 
    //     cout << "dx: " << dx.norm() << endl;
    // }







    // visualization
    // Matrix<double,Dynamic,1> x(m-1);
    // x.setZero();
    // x(0) = 0.3;
    // x(1) = 0.3;
    // x(2) = 0.3;

    // // init X
    // Matrix<double,1,Dynamic> X(d*m);
    // X.setZero();
    // // update X
    // for (int j = 1; j < m; ++j) {
    //     int k = d*j;
    //     X(k+2) = x(j-1); // 2d
    // }

    // // Construct list of relative rotations in terms of quaternion
    // // from Euler's angle
    // std::vector<Eigen::Quaternion<double>, Eigen::aligned_allocator<Eigen::Quaternion<double>>> dQ;
    // Matrix<double,1,3> th;
    // for (int j = 0; j < BE.rows(); ++j) {
    //     th = X.segment(3*j, 3);
    //     dQ.emplace_back(
    //         Eigen::AngleAxis<double>(th(0), Eigen::Vector3d::UnitX()) *
    //         Eigen::AngleAxis<double>(th(1), Eigen::Vector3d::UnitY()) *
    //         Eigen::AngleAxis<double>(th(2), Eigen::Vector3d::UnitZ())
    //     );
    // }

    // // Forward kinematics
    // Matrix<double,Dynamic,Dynamic> T;
    // igl::forward_kinematics(C, BE, P, dQ, T);

    // // LBS
    // Matrix<double,Dynamic,Dynamic> M_d;
    // igl::lbs_matrix(V, W, M_d);

    // Matrix<double,Dynamic,Dynamic> U_d;
    // U_d = M_d*T;

    // Matrix<stan::math::var,Dynamic,1> x_var(m-1);
    // x_var.setZero();
    // x_var(0) = 0.3;
    // x_var(1) = 0.3;
    // x_var(2) = 0.3;

    // auto result = f(x_var);

    // Matrix<double,Dynamic,Dynamic> u_d_3(num_V,3);
    // Matrix<double,Dynamic,Dynamic> U_d_deformed(num_V,3);
    // Matrix<double,Dynamic,Dynamic> u_d_zero(num_V,1);

    // for (int i = 0; i < num_V; i++) {
    //     u_d_3(i,0) = u_d(2*i);
    //     u_d_3(i,1) = u_d(2*i+1);
    //     u_d_3(i,2) = 0;
    // }
    // U_d_deformed = U_d + u_d_3;
    // igl::deform_skeleton(C,BE,T,CT,BET);






    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    Matrix<stan::math::var,Dynamic,1> x_var(m-1);
    x_var.setZero();
    x_var(0) = 0.1;
    x_var(1) = 0.2;
    x_var(2) = 0.3;

    auto result = f(x_var);

    Matrix<double,Dynamic,Dynamic> u_d_target(num_V,2);
    u_d_target = u_d;


    Matrix<double, Dynamic, Dynamic> tar(dim*num_V,m-1);
    Matrix<double, Dynamic, 1> dx(m-1);
    Eigen::MatrixXd J;
    Eigen::VectorXd fx;


    // design variables: euler angle
    Matrix<double,Dynamic,1> x(m-1);
    x.setZero();
    for (int i = 0; i < m-1; i++) {
        x(i) = 0;
    }

    // gradient descent
    double tol = 1e-5;
    double diff = 100;
    double step_size = 0.01; // fixed step size

    while (diff > tol) {

        MTR_BEGIN("C++", "total");
        stan::math::jacobian(f, x, fx, J);
        MTR_END("C++", "total");  

        SimplicialLDLT<SparseMatrix<double>> solver_d(K_d);
        tar = solver_d.solve(J);
        dx = 2*tar.transpose().eval()*(u_d-u_d_target);


        // // line search
        // objfunc obj(x,dx,m,d,BE,C,P);
        // // find the optimal step length
        // int bits = std::numeric_limits<double>::digits;
        // std::pair<double, double> r = boost::math::tools::brent_find_minima(obj, 0., 100., bits);
        // std::cout.precision(std::numeric_limits<double>::digits10);

        // cout << r.first << endl;


        x = x-step_size*dx;

        // diff = 0;
        diff = dx.norm();

        cout << "x: " << x << endl; 
        cout << "dx: " << dx.norm() << endl;

    }




    // // viewer
    // viewer.append_mesh();
    // viewer.data().set_mesh(U_d, F);
    // viewer.data().set_colors(blue);
    // viewer.append_mesh();
    // viewer.data().set_mesh(U_d_deformed, F);
    // viewer.data().set_colors(orange);

    // // set_color(viewer);
    // viewer.data().set_edges(CT, BET, red);
    // viewer.data().add_points(CT, red);
    // viewer.data().show_lines = false;
    // viewer.data().line_width = 10;
    // viewer.callback_key_down = &key_down;
    // viewer.launch();


    mtr_flush();
    mtr_shutdown();


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


// SparseMatrix<double> K_d;
// K_d.resize(K.rows(),K.cols());
// for (int i = 0; i < K.outerSize(); ++i) {
//     // Iterate over inside
//     for (SparseMatrix<stan::math::var>::InnerIterator it (K,i); it; ++it) {
//         // it.row(),  it.col(), it.value()
//         K_d.coeffRef(it.row(),it.col()) = it.value().val();
//     }
// }

// Matrix<double,Dynamic,Dynamic> J1;
// compute_jacobian(angles, t1, J1); // slow

// Matrix<double,Dynamic,Dynamic> J2;
// compute_jacobian(angles, f, J2); // slow

// Matrix<double,Dynamic,Dynamic> J(dim*num_V,dim*num_V);
// J = J2-J1;

// SimplicialLDLT<SparseMatrix<double>> solver_d(K_d);
// Matrix<double, Dynamic, Dynamic> tar(dim*num_V,m);
// tar = solver_d.solve(J);


// int counter = 0;
// for (int i = 0; i < t1.size(); i++) {
//     for (int j = 0; j < angles.size(); j++) {
//         if (J(i,j) != 0) {
//             counter++;
//         } 
//     }
// }

// cout << counter << endl;

// Matrix<double, Dynamic, Dynamic> gradient(m,1);
// gradient = 2*tar.transpose().eval()*u_d;

// cout << gradient << endl;

// Matrix<stan::math::var,Dynamic,1> x(m-1);
// x(0) = 1.25877;
// x(1) = 0.968614;
// x(2) = 0.667564;

// Matrix<stan::math::var,Dynamic,1> result = f(x);
// cout << u_d.norm() << endl;


// for (int i = 0; i < 20; i++) { 

//     // design variables: euler angle
//     Matrix<stan::math::var,Dynamic,1> x(m-1);
//     x.setZero();
//     for (int j = 0; j < m-1; j++) {
//         x(j) = 0.05*i;
//     }

//     Matrix<stan::math::var,Dynamic,1> result = f(x);

//     cout << u_d.norm() << endl;

// }


// objfunc obj(x,dx);
// // find the optimal step length
// int bits = std::numeric_limits<double>::digits;
// std::pair<double, double> r = boost::math::tools::brent_find_minima(obj, 0., 100., bits);
// std::cout.precision(std::numeric_limits<double>::digits10);



// stan::math::start_nested();
// try {
//     Matrix<stan::math::var, Dynamic, 1> x_var(x.size());
//     for (int k = 0; k < x.size(); ++k)
//         x_var(k) = x(k);
//     Matrix<stan::math::var, Dynamic, 1> fx_var = f(x_var);
//     fx.resize(fx_var.size());
//     for (int i = 0; i < fx_var.size(); ++i)
//         fx(i) = fx_var(i).val();
//     J.resize(fx_var.size(), x.size());

//     MTR_BEGIN("C++", "jacobian");

//     for (int i = 0; i < fx_var.size(); ++i) {
//         if (i > 0)
//             stan::math::set_zero_all_adjoints_nested();
//         stan::math::grad(fx_var(i).vi_);
//         for (int k = 0; k < x.size(); ++k)
//             J(i, k) = x_var(k).adj();
//     }

//     MTR_END("C++", "jacobian");  

// } catch (const std::exception& e) {
//     stan::math::recover_memory_nested();
//     throw;
// }
// stan::math::recover_memory_nested();