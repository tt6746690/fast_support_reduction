#include <iostream>
#include <Eigen/Core>

#include <igl/readMESH.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/barycenter.h>
#include <igl/boundary_facets.h>

#include "element.h"
#include "normalized_device_coordinate.h"

using namespace Eigen;
using namespace std;

MatrixXd V, B, C;
MatrixXi F, Tet;
const double E_y = 5e5; // Young's modulus
const double Mu_p = 0.45; // possion ratio
const double g = 100.0;

const auto getfilepath = [](const string& name, const string& ext){ 
    return "../data/" + name + "." + ext; 
};

int main(int argc, char* argv[]) {

    igl::opengl::glfw::Viewer viewer;

    string filename = "small";
    if (argc > 1) { filename = string(argv[1]); }
    igl::readMESH(getfilepath(filename, "mesh"), V, Tet, F);
    int num_V = V.rows();

    // construct the elasticity matrix E
    MatrixXd E(6, 6);
    E << 1.0-Mu_p, Mu_p, Mu_p, 0, 0, 0,
         Mu_p, 1.0-Mu_p, Mu_p, 0, 0 ,0,
         Mu_p, Mu_p, 1.0-Mu_p, 0, 0, 0,
         0, 0, 0, 0.5-Mu_p, 0, 0,
         0, 0, 0, 0, 0.5-Mu_p, 0,
         0, 0, 0, 0, 0, 0.5-Mu_p;

    E *= E_y/((1.0+Mu_p)*(1.0-2*Mu_p));

    // extract fixed vertices
    vector<int> minVertices;
    double tolerance = (V.col(1).maxCoeff() - V.col(1).minCoeff()) * 0.05;
    double min_Y = V.col(1).minCoeff();
    for (int i = 0; i < V.rows(); i++) {
        if (abs(V(i, 1) - min_Y) < tolerance) {
            minVertices.push_back(i);
        }
    }

    // construct the global stiffness matrix K
    vector<Triplet<double>> triplets;
    VectorXd f(3*num_V); // applied loads: gravity
    f.setZero();
    Vector4i nodesIds;
    vector<Element> elements;
    for (int i = 0; i < Tet.rows(); i++) {
        nodesIds = Tet.row(i);
        Element ele(nodesIds);
        ele.constructStiffnessMatrix(V, E, triplets);

        // construct force vector f
        for (int j = 0; j < 4; j++) {
            f(3*nodesIds(j)+1) += -0.25 * g * ele.volume;
        }

        // collect the element
        elements.push_back(ele);
    }


    SparseMatrix<double> K(3*num_V, 3*num_V);
    K.setFromTriplets(triplets.begin(), triplets.end());


    // apply constraints
    for (int i = 0; i < minVertices.size(); i++) {
        int idx = minVertices[i];
        K.prune([&idx](int m, int n, double) { 
            return m!=3*idx && n!=3*idx && m!=3*idx+1 && n!=3*idx+1 && m!=3*idx+2 && n!=3*idx+2; });
        K.coeffRef(3*idx, 3*idx) = 1;
        K.coeffRef(3*idx+1, 3*idx+1) = 1;
        K.coeffRef(3*idx+2, 3*idx+2) = 1;
        f(3*idx) = 0;
        f(3*idx+1) = 0;
        f(3*idx+2) = 0;
    }

    // solve for displacements at each vertex Kd = f
	SimplicialLDLT<SparseMatrix<double>> solver(K);
	VectorXd d = solver.solve(f); // displacements

    // analyze results
    VectorXi N(num_V);
    N << VectorXi::Zero(num_V);
    for(int i = 0; i < Tet.rows(); i++) {    
        for(int j = 0; j < 4; j++) {
            N[Tet(i, j)] += 1; //get normalization values for each vertex
        }
    }


    Matrix3d S;
    VectorXd s;
    s.resize(num_V);
    MatrixXd Bi(6, 12);
    VectorXd u;
    for(int i = 0; i < Tet.rows(); i++) {
        Element ele = elements[i];
        u.resize(12);
        u << d.segment(3*Tet(i, 0), 3), d.segment(3*Tet(i, 1), 3), d.segment(3*Tet(i, 2), 3), d.segment(3*Tet(i, 3), 3);
        VectorXd res = E * ele.B * u;
        S(0,0) = res(0); S(0,1) = res(5); S(0,2) = res(4);
        S(1,0) = res(5); S(1,1) = res(1); S(1,2) = res(3);
        S(2,0) = res(4); S(2,1) = res(3); S(2,2) = res(2);
        for(int j = 0; j < 4; j++) {
            s(Tet(i, j)) += S.norm()*1.0/(N(Tet(i, j)));
        }
    }

    igl::jet(s, 0, s.maxCoeff()*0.25, C);

    // Plot the mesh
    viewer.data().set_mesh(V, F);
    viewer.data().set_colors(C);
    viewer.launch();
}