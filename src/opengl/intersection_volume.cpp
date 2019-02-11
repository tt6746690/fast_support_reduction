#include "intersection_volume.h"
#include <igl/opengl/init_render_to_texture.h>
#include <igl/opengl/create_shader_program.h>
#include <igl/opengl/report_gl_error.h>
#include <igl/opengl/vertex_array.h>
#include <igl/png/render_to_png.h>
#include <igl/look_at.h>
#include <igl/ortho.h>

#include <cstdlib>
#include <iostream>

#include "normalized_device_coordinate.h"
#include "depthbuffer_to_png.h"

SelfIntersectionVolume::SelfIntersectionVolume(
    const Eigen::MatrixXf& V,
    const Eigen::MatrixXi& F,
    float width,
    float height,
    std::string shader_dir)
    :   width(width), height(height),
        V(V), F(F),
        peel_shader(
            {shader_dir+"peel.vs"}, {shader_dir+"peel.fs"}),
        intersection_shader(
            {shader_dir+"intersection.vs"}, {shader_dir+"intersection.fs"})
{
    max_passes = 20;
    done_preparation = false;
    save_png = true;
    recompile = true;
    peel_fbo  = new GLuint[3];
    peel_tex  = new GLuint[3];
    peel_dtex = new GLuint[3];
    ren_fbo   = new GLuint[2];
    ren_tex   = new GLuint[2];
    ren_dtex  = new GLuint[2];
}

SelfIntersectionVolume::~SelfIntersectionVolume()
{
    if (done_preparation) {
        glDeleteVertexArrays(1, &vao);
        glDeleteBuffers(1, &vbo);
        glDeleteBuffers(1, &ebo);
        glDeleteFramebuffers(3, peel_fbo);
        glDeleteTextures(3, peel_tex);
        glDeleteTextures(3, peel_dtex);
        glDeleteFramebuffers(2, ren_fbo);
        glDeleteTextures(2, ren_tex);
        glDeleteRenderbuffers(2, ren_dtex);
        glDeleteQueries(1, &query_id);

    }
    delete [] peel_fbo;
    delete [] peel_tex;
    delete [] peel_dtex;
    delete [] ren_fbo;
    delete [] ren_tex;
    delete [] ren_dtex;
}


void SelfIntersectionVolume::prepare() 
{
    float half = 1.001;
    igl::ortho(-half, half, -half, half, 0, 2*half, projection);
    Eigen::Vector3f eye(0, 0, -half);
    Eigen::Vector3f target(0, 0, 0);
    Eigen::Vector3f up(0, 1, 0);
    igl::look_at(eye, target, up, view.matrix());
    model = Eigen::Affine3f::Identity();

    peel_shader.compile();
    intersection_shader.compile();

    igl::opengl::vertex_array(V, F, vao, vbo, ebo);
    quad.create_vertex_array();

    for (int i = 0; i < 3; ++i) {
        igl::opengl::init_render_to_texture(width, height, true, peel_tex[i], peel_fbo[i], peel_dtex[i]);
    }
    for (int i = 0; i < 2; ++i) {
        igl::opengl::init_render_to_texture(width, height, false, ren_tex[i], ren_fbo[i], ren_dtex[i]);
    }

    for (int i = 0; i < 2; ++i) {
        glBindFramebuffer(GL_FRAMEBUFFER, ren_fbo[i]);
        glClearColor(0, 0, 0, 1.);
        glClear(GL_COLOR_BUFFER_BIT);
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    glGenQueries(1, &query_id);
    done_preparation = true;
}


void SelfIntersectionVolume::compute()
{
    int which_pass;
    GLuint any_samples_passed = 0;
    Eigen::Matrix4f mvp = (projection*view*model).matrix();

    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);
    glDepthRange(0, 1.0);
    glDisable(GL_CULL_FACE);
    glViewport(0, 0, width, height);
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    {
        peel_shader.use();
        peel_shader.set_float("width", width);
        peel_shader.set_float("height", height);
        peel_shader.set_mat4("model_view_proj", mvp);
    }

    for (int pass = 0; pass < max_passes; ++pass) 
    {
        switch(pass) {
            case 0:  which_pass = 0; break;
            case 1:  which_pass = 1; break;
            default: which_pass = 2;
        }

        // depth peeling

        glBeginQuery(GL_ANY_SAMPLES_PASSED, query_id);

        glBindFramebuffer(GL_FRAMEBUFFER, peel_fbo[pass%3]);
        glDepthFunc(GL_LESS);
        glClearColor(1, 1, 1, 1);
        glClearDepth(1);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        if (recompile) peel_shader.compile();
        peel_shader.use();
        peel_shader.set_int("which_pass", which_pass);

        if (!(which_pass == 0)) 
        {
            peel_shader.set_int("prev_depth_texture", 0);
            glActiveTexture(GL_TEXTURE0+0);
            glBindTexture(GL_TEXTURE_2D, peel_dtex[(pass-1)%3]);
        }

        glBindVertexArray(vao);
        glDrawElements(GL_TRIANGLES, F.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);

        if (save_png) {
            igl::png::render_to_png(
                std::string("peel_color_"+std::to_string(pass)+".png"), width, height, true, false);
            depthbuffer_to_png(
                std::string("peel_depth_"+std::to_string(pass)+".png"), width, height);
        }

        glEndQuery(GL_ANY_SAMPLES_PASSED);
        glGetQueryObjectuiv(query_id, GL_QUERY_RESULT, &any_samples_passed);
        
        if (any_samples_passed != 1) {
            std::cout<<"every layer peeled in "<<pass+1<<" passes"<<std::endl;
            break;
        }

        // compute self-intersection

        if (which_pass != 2)
            continue;

        glBindFramebuffer(GL_FRAMEBUFFER, ren_fbo[pass%2]);
        glDepthFunc(GL_ALWAYS);

        if (recompile) intersection_shader.compile();
        intersection_shader.use();

        intersection_shader.set_int("cur_depth_texture", 0);
        glActiveTexture(GL_TEXTURE0+0);
        glBindTexture(GL_TEXTURE_2D, peel_dtex[(pass-0)%3]);
        intersection_shader.set_int("prev_depth_texture", 1);
        glActiveTexture(GL_TEXTURE0+1);
        glBindTexture(GL_TEXTURE_2D, peel_dtex[(pass-1)%3]);
        intersection_shader.set_int("prevprev_depth_texture", 2);
        glActiveTexture(GL_TEXTURE0+2);
        glBindTexture(GL_TEXTURE_2D, peel_dtex[(pass-2)%3]);
    
        intersection_shader.set_int("cur_color_texture", 3);
        glActiveTexture(GL_TEXTURE0+3);
        glBindTexture(GL_TEXTURE_2D, peel_tex[(pass-0)%3]);
        intersection_shader.set_int("prev_color_texture", 4);
        glActiveTexture(GL_TEXTURE0+4);
        glBindTexture(GL_TEXTURE_2D, peel_tex[(pass-1)%3]);
        intersection_shader.set_int("prevprev_color_texture", 5);
        glActiveTexture(GL_TEXTURE0+5);
        glBindTexture(GL_TEXTURE_2D, peel_tex[(pass-2)%3]);

        intersection_shader.set_int("acc_color_texture", 6);
        glActiveTexture(GL_TEXTURE0+6);
        glBindTexture(GL_TEXTURE_2D, ren_tex[(pass-1)%2]);

        quad.draw();

        if (save_png) {
            igl::png::render_to_png(
                std::string("intersection_color_"+std::to_string(pass)+".png"), width, height, true, false);
            depthbuffer_to_png(
                std::string("intersection_depth_"+std::to_string(pass)+".png"), width, height);
        }
    }

    
}