#pragma once

#include <glad/glad.h>

// wraps igl::vertex_array
//      array buffer and array element buffer handle not needed
template <
    typename DerivedV,
    typename DerivedF>
void vertex_array(
    const Eigen::PlainObjectBase<DerivedV> & V,
    const Eigen::PlainObjectBase<DerivedF> & F,
    GLuint& vao);


// implementation 

#include <igl/opengl/vertex_array.h>
#include <igl/opengl/report_gl_error.h>

template <
    typename DerivedV,
    typename DerivedF>
void vertex_array(
    const Eigen::PlainObjectBase<DerivedV> & V,
    const Eigen::PlainObjectBase<DerivedF> & F,
    GLuint& vao)
{
    GLuint vbo, ebo;
    igl::opengl::vertex_array(V, F, vao, vbo, ebo);
}
