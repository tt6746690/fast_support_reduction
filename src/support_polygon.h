#pragma once
#include <Eigen/Core>
#include <igl/copyleft/cgal/convex_hull.h>
#include <vector>
#include <cassert>


// Compute convex hull of support polygon
//      in `proj_dim` direction
//      Outputs mesh of support polygon
template <
    typename DerivedV,
    typename DerivedsV,
    typename DerivedsF>
void support_polygon(
    const Eigen::MatrixBase<DerivedV>& V,
    int proj_dim,
    Eigen::PlainObjectBase<DerivedsV>& sV,
    Eigen::PlainObjectBase<DerivedsF>& sF) {

    assert(proj_dim >= 0 && proj_dim <= 2);

    std::vector<int> min_vertices;
    double tolerance = (V.col(proj_dim).maxCoeff() - V.col(proj_dim).minCoeff()) * 0.02;
    double min = V.col(proj_dim).minCoeff();
    for (int i = 0; i < V.rows(); ++i) {
        if (abs(V(i,proj_dim)-min) < tolerance) {
            min_vertices.push_back(i);
        }
    }

    Eigen::Matrix<typename DerivedV::Scalar, Eigen::Dynamic, Eigen::Dynamic> minV(min_vertices.size(), 3);
    for (int i = 0; i < min_vertices.size(); ++i) {
        minV.row(i) = V.row(min_vertices[i]);
    }

    // project along proj_dim
    minV.col(proj_dim).setConstant(min);

    igl::copyleft::cgal::convex_hull(minV, sV, sF);
}