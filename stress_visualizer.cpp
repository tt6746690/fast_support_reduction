#include <iostream>
#include <Eigen/Core>

#include <igl/readMESH.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/barycenter.h>
#include <igl/boundary_facets.h>

using namespace Eigen;
using namespace std;

MatrixXd V, B, C;
MatrixXi F, Tet;

const auto getfilepath = [](const string& name, const string& ext){ 
    return "../data/" + name + "." + ext; 
};

int main(int argc, char* argv[]) {
    string filename = "small";
    if (argc > 1) { filename = string(argv[1]); }
    igl::readMESH(getfilepath(filename, "mesh"), V, Tet, F);



    Eigen::VectorXd Z = -V.col(1);
    igl::jet(Z, true, C);

    // Plot the mesh
    igl::opengl::glfw::Viewer viewer;
    viewer.data().set_mesh(V, F);
    viewer.data().set_colors(C);
    viewer.launch();
}