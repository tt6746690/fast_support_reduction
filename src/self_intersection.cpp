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
    return e1.p1.coord[1] < e2.p1.coord[1];
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

bool projection_below(
    const Eigen::VectorXd v1,
    const Eigen::VectorXd v2,
    const Eigen::VectorXd v3)
{
    double y = calculate_y(v1, v2, v3[0]);
    return y < v3[1];
}

bool going_in(
    const Eigen::VectorXd v1,
    const Eigen::VectorXd v2,
    const Eigen::VectorXd v3)
{
    if ((v3[1] > v1[1] && v3[1] > v2[1]) || 
        projection_below(v1, v2, v3))
    {
        return true;
    }
    
    return false;
}

int third_vertex(
    const Eigen::MatrixXi F, 
    int v1, 
    int v2)
{
    for (int i = 0; i < F.size(); i++) 
    {
        Eigen::VectorXi f = F.row(i);
        if ((v1 == f[0] || v1 == f[1] || v1 == f[2]) &&
            (v2 == f[0] || v2 == f[1] || v2 == f[2])) 
        {
            for (int j = 0; j < 2; j++) 
            {
                if (v1 != f[j] && v2 != f[j]) 
                {
                    return f[j];
                }
            }
        }
    }

    return -1;
}

double self_intersection(
    const Eigen::MatrixXd V,
    const Eigen::MatrixXi F) 
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
        e.p1 = p1.coord[1] < p2.coord[1] ? p1 : p2;
        e.p2 = p1.coord == e.p1.coord ? p2 : p1;

        boundary_vertices.push_back(V.row(loop[i]));
        boundary_edges.push_back(e);
    }

    std::sort(boundary_vertices.begin(), boundary_vertices.end(), sort_by_x);
    std::sort(boundary_edges.begin(), boundary_edges.end(), sort_by_y);

    double area = 0;
    double y = NAN;

    for (int i = 0; i < boundary_vertices.size() - 1; i++)
    {
        double x = 0.5 * ((boundary_vertices[i])[0] + (boundary_vertices[i + 1])[0]);
        int expected_dir = IN;
        int levels_deep = 0;
        
        for (int j = 0; j < boundary_edges.size(); j++) 
        {
            Edge e = boundary_edges[j];
            Eigen::VectorXd v1 = e.p1.coord;
            Eigen::VectorXd v2 = e.p2.coord;

            if (((x >= v1[0] && x <= v2[0]) || (x <= v1[0] && x >= v2[0])) &&
                (v1[0] != v2[0]))
            {
                Eigen::VectorXd v3 = V.row(third_vertex(F, e.p1.index, e.p2.index));
                int actual_dir = going_in(v1, v2, v3) ? IN : OUT;
                levels_deep += actual_dir;

                if (expected_dir != actual_dir) 
                {
                    if (y == NAN) 
                    {
                        y = calculate_y(v1, v2, x);
                    }
                    else 
                    {
                        int new_y = calculate_y(v1, v2, x);
                        area += new_y - y;
                        y = new_y;
                    }
                }
                else if (y != NAN) 
                {
                    area += calculate_y(v1, v2, x) - y;
                    y = NAN;
                }

                expected_dir = (0 - levels_deep) / abs(levels_deep);
            }
        }
    }

    return area;
}