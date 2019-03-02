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

    vector<Element> elements;

    // construct the global stiffness matrix K
    vector<Triplet<double>> triplets;
    VectorXd f(3*num_V); // applied loads: gravity
    f << VectorXd::Zero(3*num_V);
    Vector4i nodesIds;
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

    // solve for displacements at each vertex Kd = f
	SimplicialLDLT<SparseMatrix<double>> solver(K);
	VectorXd d = solver.solve(f); // displacements

    // analyze results
    VectorXd N(num_V);
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

    igl::jet(s, true, C);

    // Plot the mesh
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.data().set_colors(C);
    viewer.launch();
}