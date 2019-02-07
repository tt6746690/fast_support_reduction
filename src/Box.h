#pragma once

#include <Eigen/Core>
#include <glad/glad.h>

template <typename T>
class Box {
    using MatrixXTR = Eigen::Matrix<T,   Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    using MatrixXIR = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
public:
    Box(double half_width);
    ~Box();
    void create_vertex_array();
    void draw();
public:
    double half_width;
    MatrixXTR V;
    MatrixXIR F;

    bool vao_created;
    GLuint vao;
    GLuint vbo;
    GLuint ebo;
};



// implementations

#include <iostream>
#include <cassert>
#include <igl/opengl/vertex_array.h>

template <typename T>
Box<T>::Box(double half_width)
    : half_width(half_width), vao_created(false), vao(0), vbo(0), ebo(0)
{
    auto h = static_cast<T>(half_width);
    V.resize(8, 3);
    F.resize(4, 4);
    V <<
        -h, -h, -h,
        h, -h, -h,
        h, h, -h,
        -h, h, -h,
        -h, -h, h,
        h, -h, h,
        h, h, h,
        -h, h, h;

    F <<
        0, 1, 2, 3,     // floor 
        4, 5, 6, 7,     // roof
        0, 4, 1, 5,     // pillars
        2, 6, 3, 7;
}

template <typename T>
Box<T>::~Box()
{
    if (vao_created) {
        glDeleteVertexArrays(1, &vao);
        glDeleteBuffers(1, &vbo);
        glDeleteBuffers(1, &ebo);
    }
}

template <typename T>
void Box<T>::create_vertex_array()
{
    if (!vao_created) {
        igl::opengl::vertex_array(V, F, vao, vbo, ebo);
        vao_created = true;
    }
}

template <typename T>
void Box<T>::draw()
{
    assert(vao_created == true);
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glBindVertexArray(vao);
    glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_INT, 0);
    glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_INT, (void*)(4*sizeof(GLuint)));
    glDrawElements(GL_LINES, 8, GL_UNSIGNED_INT, (void*)(8*sizeof(GLuint)));
    glBindVertexArray(0);
}