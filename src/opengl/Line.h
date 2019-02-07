#pragma once

#include <Eigen/Core>
#include <glad/glad.h>

template <typename T>
class Line {
    using Vector3T = Eigen::Matrix<T, 3, 1>;
    using MatrixXTR = Eigen::Matrix<T,   Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    using MatrixXIR = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
public:
    Line(Vector3T&& start, Vector3T&& end);
    ~Line();
    void create_vertex_array();
    void draw();
public:
    Vector3T start;
    Vector3T end;
    MatrixXTR V;
    MatrixXIR F;

    bool vao_created;
    GLuint vao;
    GLuint vbo;
    GLuint ebo;
};

// implementations

#include <cassert>
#include <igl/opengl/vertex_array.h>

template <typename T>
Line<T>::Line(Vector3T&& start, Vector3T&& end)
    : start(start), end(end), vao_created(false), vao(0), vbo(0), ebo(0)
{
    V.resize(2, 3);
    F.resize(1, 2);
    V << start.transpose(), end.transpose();
    F << 0, 1;
}

template <typename T>
Line<T>::~Line()
{
    if (vao_created) {
        glDeleteVertexArrays(1, &vao);
        glDeleteBuffers(1, &vbo);
        glDeleteBuffers(1, &ebo);
    }
}

template <typename T>
void Line<T>::create_vertex_array()
{
    if (!vao_created) {
        igl::opengl::vertex_array(V, F, vao, vbo, ebo);
        vao_created = true;
    }
}


template <typename T>
void Line<T>::draw()
{
    assert(vao_created == true);
    glBindVertexArray(vao);
    glDrawElements(GL_LINES, F.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}