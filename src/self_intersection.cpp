#include "self_intersection.h"
#include <igl/boundary_loop.h>

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

double calculate_y(
    const Eigen::VectorXd v1,
    const Eigen::VectorXd v2,
    double x) 
{
    double m = (v2[1] - v1[1]) / (v2[0] - v1[0]);
    double b = v1[1] - m * v1[0];
    return m * x + b;
}

double self_intersection(
    const Eigen::MatrixXd V,
    const Eigen::MatrixXi F,
    std::vector<Eigen::Vector3d> & intersection) 
{
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
                        area += new_y - y;
                        intersection.push_back(Eigen::Vector3d(x, y, 0));
                        intersection.push_back(Eigen::Vector3d(x, new_y, 0));
                        y = new_y;
                    }
                }
                else if (abs(levels_deep) == 1 && abs(prev_depth) != 0)
                {
                    int new_y = calculate_y(v1, v2, x);
                    area += new_y - y;
                    intersection.push_back(Eigen::Vector3d(x, y, 0));
                    intersection.push_back(Eigen::Vector3d(x, new_y, 0));
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