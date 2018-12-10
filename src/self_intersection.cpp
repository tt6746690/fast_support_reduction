#include "self_intersection.h"
#include "minitrace.h"

#include <igl/boundary_loop.h>
#include <igl/per_face_normals.h>
#include <tuple>
#include <Eigen/Sparse>

struct Point {
    Eigen::VectorXd coord;
    int index;
};

struct Edge {
    Point p1;
    Point p2;
};

bool sort_by_x(
    const Eigen::VectorXd v1, 
    const Eigen::VectorXd v2) 
{
    return v1[0] < v2[0];
}

bool sort_by_y(
    const Edge e1,
    const Edge e2)
{
    return std::min(e1.p1.coord[1], e1.p2.coord[1]) < std::min(e2.p1.coord[1], e2.p2.coord[1]);
}

bool sort_by_first(
    const std::tuple<double, int> t1,
    const std::tuple<double, int> t2) 
{
    return std::get<0>(t1) < std::get<0>(t2);
}

double calculate_y(
    const Eigen::VectorXd v1,
    const Eigen::VectorXd v2,
    double x) 
{
    double m = (v2[1] - v1[1]) / (v2[0] - v1[0]);
    double b = v1[1] - m * v1[0];
    return m * x + b;
}

void surrounding_points(
    Eigen::Vector3d v1,
    Eigen::Vector3d v2,
    Eigen::Vector3d v3,
    Eigen::Vector3d corner,
    double step,
    int & x1,
    int & x2,
    int & y1,
    int & y2) 
{
    x1 = floor((std::min(v1[0], std::min(v2[0], v3[0])) - corner[0]) / step);
    x2 = floor((std::max(v1[0], std::max(v2[0], v3[0])) - corner[0]) / step);
    y1 = floor((std::min(v1[1], std::min(v2[1], v3[1])) - corner[1]) / step);
    y2 = floor((std::max(v1[1], std::max(v2[1], v3[1])) - corner[1]) / step);
}

bool point_in_triangle(
    Eigen::Vector3d v1,
    Eigen::Vector3d v2,
    Eigen::Vector3d v3,
    Eigen::Vector3d p,
    double & alpha,
    double & beta,
    double & gamma)
{
    // barycentric coordinates
    alpha = ((v2[1] - v3[1])*(p[0] - v3[0]) + (v3[0] - v2[0])*(p[1] - v3[1])) /
        ((v2[1] - v3[1])*(v1[0] - v3[0]) + (v3[0] - v2[0])*(v1[1] - v3[1]));
    beta = ((v3[1] - v1[1])*(p[0] - v3[0]) + (v1[0] - v3[0])*(p[1] - v3[1])) /
        ((v2[1] - v3[1])*(v1[0] - v3[0]) + (v3[0] - v2[0])*(v1[1] - v3[1]));
    gamma = 1 - alpha - beta;

    return alpha > 0 && beta > 0 && gamma > 0;
}

std::vector<std::tuple<int, double>> grid_points_in_triangle(
    Eigen::Vector3d v1,
    Eigen::Vector3d v2,
    Eigen::Vector3d v3,
    double step,
    Eigen::Vector3d corner,
    int nx,
    int ny) 
{
    int x1, x2, y1, y2;
    surrounding_points(v1, v2, v3, corner, step, x1, x2, y1, y2);

    int cur_x = x1;
    std::vector<std::tuple<int, double>> points_in_triangle;

    while (y1 < y2) 
    {
        while (cur_x < x2) 
        {
            assert(cur_x < nx);
            assert(y1 < ny);

            Eigen::Vector3d cur_pos(cur_x * step + corner[0], y1 * step + corner[1], 0);
            double alpha, beta, gamma;
            if (point_in_triangle(v1, v2, v3, cur_pos, alpha, beta, gamma)) 
            {
                Eigen::Vector3d p = alpha * v1 + beta * v2 + gamma * v3;
                int idx = y1 * nx + cur_x;
                points_in_triangle.push_back(std::tuple<int, double>(idx, p[2]));
            }

            cur_x++;
        }

        cur_x = x1;
        y1++;
    }

    return  points_in_triangle;
}

double self_intersection_2d(
    const Eigen::MatrixXf Vf,
    const Eigen::MatrixXi F)
{
    MTR_SCOPE_FUNC();

    Eigen::MatrixXd V = Vf.cast<double>();
    Eigen::VectorXi loop;
    igl::boundary_loop(F, loop);

    std::vector<Eigen::VectorXd> boundary_vertices;
    std::vector<Edge> boundary_edges;
    for (int i = 0; i < loop.size(); i++) 
    {
        Point p1, p2;
        p1.index = loop[i];
        p1.coord = V.row(p1.index);
        p2.index = loop[(i + 1) % loop.size()];
        p2.coord = V.row(p2.index);
        
        Edge e;
        e.p1 = p1;
        e.p2 = p2;

        boundary_vertices.push_back(V.row(loop[i]));
        boundary_edges.push_back(e);
    }

    std::sort(boundary_vertices.begin(), boundary_vertices.end(), sort_by_x);
    std::sort(boundary_edges.begin(), boundary_edges.end(), sort_by_y);

    double area = 0;
    double y;

    for (int i = 0; i < boundary_vertices.size() - 1; i++)
    {
        double x = 0.5 * ((boundary_vertices[i])[0] + (boundary_vertices[i + 1])[0]);
        int expected_dir;
        int levels_deep = 0;
        int prev_depth = 0;

        for (int j = 0; j < boundary_edges.size(); j++) 
        {
            Edge e = boundary_edges[j];
            Eigen::VectorXd v1 = e.p1.coord;
            Eigen::VectorXd v2 = e.p2.coord;

            // are we crossing?
            if (((x >= v1[0] && x <= v2[0]) || (x <= v1[0] && x >= v2[0])) &&
                (v1[0] != v2[0]))
            {
                int actual_dir = v1[0] > v2[0] ? POS : NEG;
                prev_depth = levels_deep;
                levels_deep += actual_dir;

                // first time crossing our shape
                if (abs(levels_deep) == 1 && abs(prev_depth == 0))
                {
                    // set expected dir to be opposite actual dir for next iteration
                    expected_dir = -1 * actual_dir;
                }

                else if (expected_dir != actual_dir) 
                {
                    // just entered self-intersection
                    if (abs(levels_deep) == 2) 
                    {
                        y = calculate_y(v1, v2, x);
                    }
                    else 
                    {
                        int new_y = calculate_y(v1, v2, x);
                        // need absolute value since order of edges can be a little off
                        area += abs(new_y - y);
                        y = new_y;
                    }
                }
                else if (abs(levels_deep) == 1 && abs(prev_depth) != 0)
                {
                    int new_y = calculate_y(v1, v2, x);
                    area += abs(new_y - y);
                    y = new_y;
                }

                if (levels_deep != 0) 
                {
                    expected_dir = (0 - levels_deep) / abs(levels_deep);
                }
            }
        }
    }

    return area;
}

double self_intersection_3d(
    const Eigen::MatrixXf Vf,
    const Eigen::MatrixXi F)
{
    MTR_SCOPE_FUNC();

    Eigen::MatrixXd V = Vf.cast<double>();
    Eigen::MatrixXd N;
    igl::per_face_normals(V, F, N);

    // define grid
    double x_min = V.col(0).minCoeff();
    double x_max = V.col(0).maxCoeff();
    double delta = (x_max - x_min) / 1000;
    x_min -= delta;
    x_max += delta;
    double y_min = V.col(1).minCoeff() - delta;
    double y_max = V.col(1).maxCoeff() + delta;
    double z_min = V.col(2).minCoeff();
    double step = sqrt((x_max - x_min) * (y_max - y_min) / (F.rows() * 8));
    int nx = round((x_max - x_min) / step);
    int ny = round((y_max - y_min) / step);
    Eigen::Vector3d corner(x_min, y_min, z_min);

    Eigen::SparseMatrix<double> hits(F.rows(), nx * ny);
    typedef Eigen::Triplet<double> T;
    std::vector<T> triplets;
    triplets.reserve(F.rows());

    for (int i = 0; i < F.rows(); i++) 
    {
        Eigen::RowVector3i f = F.row(i);
        std::vector<std::tuple<int, double>> points_in_triangle = grid_points_in_triangle(
            V.row(f[0]), V.row(f[1]), V.row(f[2]), step, corner, nx, ny);

        for (int j = 0; j < points_in_triangle.size(); j++) {
            std::tuple<int, double> t = points_in_triangle[j];
            triplets.push_back(T(i, std::get<0>(t), std::get<1>(t)));
        }
    }

    hits.setFromTriplets(triplets.begin(), triplets.end());

    double volume = 0;
    double z;

    for (int i = 0; i < hits.outerSize(); i++)
    {
        std::vector<std::tuple<double, int>> zs;
        for (Eigen::SparseMatrix<double>::InnerIterator it(hits, i); it; ++it)
        {
            zs.push_back(std::tuple<double, int>(it.value(), it.row()));
        }

        std::sort(zs.begin(), zs.end(), sort_by_first);
        int cur_depth = 0;
        int prev_depth = 0;
        int expected_dir = IN;

        for (int j = 0; j < zs.size(); j++) 
        {
            if (zs.size() % 2 == 1) { continue; }

            Eigen::RowVector3d n = N.row(std::get<1>(zs[j]));
            int actual_dir = n[2] / abs(n[2]);
            prev_depth = cur_depth;
            cur_depth += actual_dir;

            if (abs(cur_depth) == 2 && abs(prev_depth) == 1) 
            {
                z = std::get<0>(zs[j]);
            }
            else if (abs(cur_depth) == 1 && abs(prev_depth) == 2) 
            {
                double new_z = std::get<0>(zs[j]);
                volume += new_z - z;
            }

            expected_dir = cur_depth == 0 ? IN : -cur_depth / abs(cur_depth);
        }
    }

    return volume;
}