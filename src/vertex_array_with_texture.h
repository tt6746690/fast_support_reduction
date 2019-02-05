#pragma once

#include <glad/glad.h>


// Create GL_VERTEX_ARRAY for a given mesh (V, F) and texture coordinates
//      specifically hardcoded for off-screen rendering in depth peeling
//
// Memory layout:
//          x y z s t (r)
//
// Usage in shaders
//          layout(location = 0) in  vec3 pos_vs_in;
//          layout(location = 1) in  vec2 tex_coord;
//
// Inputs:
//      V   #V by dim list of mesh vertex positions
//      F   #F by 3 list of mesh triangle indices into V
//      T   #V by dim list of texture coordinates
//  
// Outputs:
//      vao     vertex array buffer object handle
//
//  Note: might need to return vbo,ebo for `glDeleteBuffers` later
template <
    typename DerivedV,
    typename DerivedF,
    typename DerivedT>
void vertex_array_texture(
    const Eigen::PlainObjectBase<DerivedV>& V,
    const Eigen::PlainObjectBase<DerivedF>& F,
    const Eigen::PlainObjectBase<DerivedT>& T,
    GLuint& vao);


// implementations

#include <type_traits>

template <
    typename DerivedV,
    typename DerivedF,
    typename DerivedT>
void vertex_array_texture(
    const Eigen::PlainObjectBase<DerivedV>& V,
    const Eigen::PlainObjectBase<DerivedF>& F,
    const Eigen::PlainObjectBase<DerivedT>& T,
    GLuint& vao)
{

    // Inputs should be in RowMajor storage. 
    //      If not, we have no choice but to create a copy.
    if(!(V.Options & Eigen::RowMajor))
    {
        Eigen::Matrix<
            typename DerivedV::Scalar,
            DerivedV::RowsAtCompileTime,
            DerivedV::ColsAtCompileTime,
            Eigen::RowMajor> VR = V;
        return vertex_array_texture(VR, F, T, vao);
    }
    if(!(F.Options & Eigen::RowMajor))
    {
        Eigen::Matrix<
            typename DerivedF::Scalar,
            DerivedF::RowsAtCompileTime,
            DerivedF::ColsAtCompileTime,
            Eigen::RowMajor> FR = F;
        return vertex_array_texture(V, FR, T, vao);
    }
    if(!(T.Options & Eigen::RowMajor))
    {
        Eigen::Matrix<
            typename DerivedT::Scalar,
            DerivedT::RowsAtCompileTime,
            DerivedT::ColsAtCompileTime,
            Eigen::RowMajor> TR = T;
        return vertex_array_texture(V, F, TR, vao);
    }

    assert((std::is_same<typename DerivedV::Scalar, typename DerivedT::Scalar>::value) &&
        "vertex position and texture coordinate type should be same");
    assert(V.rows() == T.rows() && "vertex position and texture coordinate should have same size");

    Eigen::Matrix<
        typename DerivedV::Scalar,
        DerivedV::RowsAtCompileTime,
        DerivedV::ColsAtCompileTime,
        Eigen::RowMajor> VT(V.rows(), V.cols()+T.cols());
    VT << V, T;

    const auto size_VTScalar = sizeof(typename DerivedV::Scalar);
    const auto size_FScalar  = sizeof(typename DerivedF::Scalar);

    assert(sizeof(GLuint) == size_FScalar && "F type does not match GLuint");

    GLuint vbo, ebo;
    glGenVertexArrays(1, &vao);
    glGenBuffers(1, &vbo);
    glGenBuffers(1, &ebo);
    glBindVertexArray(vao);

    glBindBuffer(GL_ARRAY_BUFFER, vbo);
    glBufferData(GL_ARRAY_BUFFER, size_VTScalar*VT.size(), VT.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, size_FScalar*F.size(), F.data(), GL_STATIC_DRAW);

    GLsizei stride = static_cast<GLsizei>((V.cols()+T.cols())*size_VTScalar);
    glVertexAttribPointer(0, V.cols(), 
        size_VTScalar==sizeof(float)?GL_FLOAT:GL_DOUBLE, GL_FALSE, stride,
        (GLvoid*)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, T.cols(),
        size_VTScalar==sizeof(float)?GL_FLOAT:GL_DOUBLE, GL_FALSE, stride,
        (GLvoid*)(V.cols()*size_VTScalar));
    glEnableVertexAttribArray(1);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}
