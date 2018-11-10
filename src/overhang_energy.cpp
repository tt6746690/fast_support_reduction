#include "overhang_energy.h"

#include <igl/slice.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/boundary_loop.h>
#include <igl/slice.h>

#include <vector>
#include <algorithm>
#include <cmath>
#include <iostream>

double overhang_energy(
    const Eigen::MatrixXd V,
    const Eigen::MatrixXi F,
    const Eigen::RowVector3d dp,
    const double tau,
    bool display)
{

    Eigen::RowVector3d dpn;
    dpn = dp.normalized();

    double e;
    double energy = 0;

    // 2D case

    if (V.cols() == 3 && V.col(2).sum() == 0.) {

        Eigen::VectorXi bnd;
        igl::boundary_loop(F, bnd);

        int bnd_size = bnd.size();
        Eigen::RowVector3d i, j, ele, n;
        std::vector<int> unsafe;

        for (int b = 0; b < bnd_size; ++b) {
            i = V.row(bnd(b));
            j = V.row(bnd((b+1)%bnd_size));
            ele = (i - j).normalized();
            n(0) = -ele(1);
            n(1) =  ele(0);
            n(2) =  ele(2);
            e = n.dot(dpn) + tau;
            if (e < 0) {
                unsafe.push_back(bnd(b));
                unsafe.push_back(bnd((b+1)%bnd_size));
            }
            e = std::pow(std::min(e, 0.), 2.0);
            energy += e;
        }

        if (display) {
            std::cout << "energy: " << energy << '\n';
            // Plot the mesh with pseudocolors
            const Eigen::RowVector3d red(255./255., 0., 0.);
            const Eigen::RowVector3d green(0., 255./255., 0.);
            igl::opengl::glfw::Viewer viewer;
            viewer.data().set_mesh(V, F);
            // draw unsafe edges
            for (int i = 0; i < unsafe.size() / 2; ++i) {
                viewer.data().add_edges(
                    V.row(unsafe[2*i]),
                    V.row(unsafe[2*i+1]),
                    red
                );
            }
            viewer.data().show_lines = false;
            viewer.launch();
        }

    } else {
        printf("overlap energy for 3D not supported!\n");
    }

    return energy;
}