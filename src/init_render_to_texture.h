#pragma once

#include <glad/glad.h>

// create frame buffer that renders color to RGBA texture and depth to depth texture
void init_render_to_texture(
    const int width,
    const int height,
    GLuint& fbo,
    GLuint& color_tex,
    GLuint& depth_tex);