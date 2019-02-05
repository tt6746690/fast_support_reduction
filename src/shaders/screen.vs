#version 410 core

layout(location = 0) in vec3 pos_vs_in;
layout(location = 1) in vec2 tex_coord_vs_in;

out vec2 tex_coord_fs_in;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;

void main()
{
    gl_Position = proj * view * model * vec4(pos_vs_in, 1);
    tex_coord_fs_in = tex_coord_vs_in;
}