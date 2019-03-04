// Because of Mosek complications, we don't use static library if Mosek is used.
#ifdef LIBIGL_WITH_MOSEK
#ifdef IGL_STATIC_LIBRARY
#undef IGL_STATIC_LIBRARY
#endif
#endif

#include <igl/copyleft/tetgen/mesh_with_skeleton.h>
#include <igl/boundary_conditions.h>
#include <igl/colon.h>
#include <igl/directed_edge_parents.h>
#include <igl/jet.h>
#include <igl/readDMAT.h>
#include <igl/readMESH.h>
#include <igl/readTGF.h>
#include <igl/readOBJ.h>
#include <igl/writeOBJ.h>
#include <igl/writeMESH.h>
#include <igl/writeDMAT.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/bbw.h>
#include <igl/decimate.h>

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <vector>
#include <algorithm>
#include <iostream>

#include "robust_bbw.h"

const Eigen::RowVector3d sea_green(70./255.,252./255.,167./255.);
int selected = 0;
Eigen::MatrixXd V, U, W, TW, C, TV;
Eigen::MatrixXi TT, G, F, BE;
Eigen::VectorXi P, J;

const auto getfilepath = [](const std::string& path, const std::string& name, const std::string& ext){ 
    return path + name + "." + ext; 
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
    case '.':
      selected++;
      selected = std::min(std::max(selected,0),(int)TW.cols()-1);
      set_color(viewer);
      break;
    case ',':
      selected--;
      selected = std::min(std::max(selected,0),(int)TW.cols()-1);
      set_color(viewer);
      break;
  }
  return true;
}

int main(int argc, char *argv[])
{
  using namespace Eigen;
  using namespace std;

  string filename = string(argv[1]);
  igl::readOBJ(getfilepath("../", filename, "obj"), U, G);

  // simplify the input mesh if too dense
  if (G.rows() > 11000) { // just in case
      igl::decimate(U, G, 10000, V, F, J);
      igl::writeOBJ(getfilepath("../", filename, "obj"), V, F);
  }
  else {
      V = U;
      F = G;
  }

  igl::readTGF(getfilepath("../", filename, "tgf"), C, BE);
  // retrieve parents for forward kinematics
  igl::directed_edge_parents(BE, P);

  // TW: tet weights; W: surface weights for visualization
  robust_bbw(V, F, C, BE, TV, TT, TW, W);

  igl::writeMESH(getfilepath("../", filename, "mesh"), TV, TT, F);
  igl::writeDMAT(getfilepath("../", filename, "dmat"), TW);

  // Plot the mesh with pseudocolors
  igl::opengl::glfw::Viewer viewer;
  viewer.data().set_mesh(V, F);
  set_color(viewer);
  viewer.data().set_edges(C, BE, sea_green);
  viewer.data().show_lines = false;
  viewer.data().show_overlay_depth = false;
  viewer.data().line_width = 1;
  viewer.callback_key_down = &key_down;
  viewer.core.is_animating = false;
  cout<<
    "Press '.' to show next weight function."<<endl<<
    "Press ',' to show previous weight function."<<endl<<
  viewer.launch();
  return EXIT_SUCCESS;
}


