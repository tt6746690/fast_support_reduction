#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glad/glad.h>

#include <string>
#include "Quad.h"
#include "Shader.h"

class SelfIntersectionVolume {
public:
    SelfIntersectionVolume(
        const Eigen::MatrixXf& V,
        const Eigen::MatrixXf& W,
        const Eigen::MatrixXi& F,
        float width,
        float height,
        std::string shader_dir = "../src/shaders/");
    ~SelfIntersectionVolume();

    // prepares for
    //      1. transformation matrices
    //      2. compile and link shaders
    //      3. create vertex array objects for the mesh
    //      4. create framebuffer for depth peeling and volume computation
    //      5. create query object for computing pre-`max_passes` break
    //      6. zero value in color buffer used to accumulate self-intersecting volume
    //  
    //  assumption: glfw window and opengl context created
    void prepare();

    // computes self-intersection volume of mesh (V, F)
    float compute();

public:
    // size of framebuffer, controls how precise computed value is 
    float width;
    float height;

    // number of bones
    int num_bones;

    // mesh data 
    Eigen::MatrixXf V;
    Eigen::MatrixXf W;
    Eigen::MatrixXi F;
    Eigen::MatrixXf T;
    
    // opengl object handles 
    GLuint vao;
    GLuint vbo;
    GLuint ebo;

    // for rendering to intersection shader 
    Quad<float> quad;

    // shader program for depth peeling
    Shader peel_shader;
    // shader program for computing self-intersection volume
    Shader intersection_shader;
    // transformation matrices
    Eigen::Matrix4f projection;
    Eigen::Affine3f view;
    Eigen::Affine3f model;
    // framebuffers     size        usage
    //      peel_*      3           color/depth buffer stores normal_dir/depth info
    //      ren_*       2           color buffer records self-intersection volume
    GLuint* peel_fbo;
    GLuint* peel_tex;
    GLuint* peel_dtex;
    GLuint* ren_fbo;
    GLuint* ren_tex;
    GLuint* ren_dtex;

    // max number of passes for depth peeling
    int max_passes;
    // query object handle
    GLuint query_id;
    // query object handle for benchmarking purposes
    GLuint query_benchmark;

    // whether `prepare()` is called, remind to do garbage collection in dtor
    bool done_preparation;
    // whether output debugging pngs
    bool save_png;
    // whether to re-compile shader code in render loop
    bool recompile;

    // to temporarily store the self-intersection volume information
    unsigned char * volume_buffer;

    float ortho_box_volume;

};

