#version 410 core

layout(location = 0) in  vec3 pos_vs_in;

out vec4 pos_fs_in;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;

void main()
{
    pos_fs_in = proj * view * model * vec4(pos_vs_in, 1);
    gl_Position = pos_fs_in;
}