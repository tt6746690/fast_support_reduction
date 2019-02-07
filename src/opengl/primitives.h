#pragma once

#include <Eigen/Core>

// generate vertices and edges for a centered square box
//      l       length of 1 side
//
//      vertex_array(V, F, vao);
//      ...
//      glBindVertexArray(vao_box);
//      glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_INT, 0);
//      glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_INT, (void*)(4*sizeof(GLuint)));
//      glDrawElements(GL_LINES, 8, GL_UNSIGNED_INT, (void*)(8*sizeof(GLuint)));
//      glBindVertexArray(0);
//
template <
    typename DerivedV,
    typename DerivedF>
void box(
    double l, 
    Eigen::PlainObjectBase<DerivedV>& V,
    Eigen::PlainObjectBase<DerivedF>& F);

// generate vertices and edges for a line
//  
//      vertex_array(V, F, vao);
//      ...
//      glBindVertexArray(vao);
//      glDrawElements(GL_LINES, F.size(), GL_UNSIGNED_INT, 0);
//      glBindVertexArray(0);
//  
template <
    typename Scalar,
    typename DerivedV,
    typename DerivedF>
void line(
    const Eigen::Matrix<Scalar, 3, 1>& start,
    const Eigen::Matrix<Scalar, 3, 1>& end,
    Eigen::PlainObjectBase<DerivedV>& V, 
    Eigen::PlainObjectBase<DerivedF>& F);


// generate vertices and edges and texture coordinates for a quad
//
//      vertex_array_texture(V, F, T, vao);
//      ...
//      glBindVertexArray(vao);
//      glDrawElements(GL_TRIANGLES, F.size(), GL_UNSIGNED_INT, 0);
//      glBindVertexArray(0);
template <
    typename DerivedV,
    typename DerivedF,
    typename DerivedT>
void textured_quad(
    Eigen::PlainObjectBase<DerivedV>& V,
    Eigen::PlainObjectBase<DerivedF>& F,
    Eigen::PlainObjectBase<DerivedT>& T);





// implementation 


template <
    typename DerivedV,
    typename DerivedF>
void box(
    double l, 
    Eigen::PlainObjectBase<DerivedV>& V,
    Eigen::PlainObjectBase<DerivedF>& F)
{
    auto h = static_cast<typename DerivedV::Scalar>(l / 2);
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
        0, 1, 2, 3, // floor 
        4, 5, 6, 7, // roof
        0, 4, 1, 5, // pillars
        2, 6, 3, 7;
}

template <
    typename Scalar,
    typename DerivedV,
    typename DerivedF>
void line(
    const Eigen::Matrix<Scalar, 3, 1>& start,
    const Eigen::Matrix<Scalar, 3, 1>& end,
    Eigen::PlainObjectBase<DerivedV>& V, 
    Eigen::PlainObjectBase<DerivedF>& F)
{
    V.resize(2, 3);
    F.resize(1, 2);

    V << start.transpose(),
         end.transpose();
    F << 0, 1;
}



template <
    typename DerivedV,
    typename DerivedF,
    typename DerivedT>
void textured_quad(
    Eigen::PlainObjectBase<DerivedV>& V,
    Eigen::PlainObjectBase<DerivedF>& F,
    Eigen::PlainObjectBase<DerivedT>& T)
{
    V.resize(4, 3);
    T.resize(4, 2);
    F.resize(2, 3);
    V << -1, -1, -1,
        -1, 1, -1,
        1, 1,  -1,
        1, -1, -1;
    T << 0, 0,
        0, 1,
        1, 1,
        1, 0;
    F << 0, 1, 2,
        0, 2, 3;
}
