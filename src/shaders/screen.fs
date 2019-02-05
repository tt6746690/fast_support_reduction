#version 410 core

// in  vec4 pos_fs_in;
in  vec2 tex_coord_fs_in;
out vec4 color;

uniform sampler2D orthoproj_tex;

void main()
{
    color = texture(orthoproj_tex, tex_coord_fs_in);
}