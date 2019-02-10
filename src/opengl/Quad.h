#pragma once

#include <Eigen/Core>
#include <glad/glad.h>

template <typename T>
class Quad {
    using MatrixXTR = Eigen::Matrix<T,   Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    using MatrixXIR = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
public:
    Quad();
    ~Quad();
    void create_vertex_array();
    void draw();
public:
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
Quad<T>::Quad()
    : vao_created(false), vao(0), vbo(0), ebo(0)
{
    V.resize(4, 3);
    F.resize(2, 3);
    V << -1, -1, 0,
          1, -1, 0,
          1,  1, 0,
         -1,  1, 0;
    F << 0, 1, 2,
         0, 2, 3;
}

template <typename T>
Quad<T>::~Quad()
{
    if (vao_created) {
        glDeleteVertexArrays(1, &vao);
        glDeleteBuffers(1, &vbo);
        glDeleteBuffers(1, &ebo);
    }
}

template <typename T>
void Quad<T>::create_vertex_array()
{
    if (!vao_created) {
        igl::opengl::vertex_array(V, F, vao, vbo, ebo);
        vao_created = true;
    }
}


template <typename T>
void Quad<T>::draw()
{
    assert(vao_created == true);
    glBindVertexArray(vao);
    glDrawElements(GL_TRIANGLES, F.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}