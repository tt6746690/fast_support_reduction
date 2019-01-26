#version 410 core

layout(location = 0) in  vec3 pos_vs_in;

out vec4 pos_fs_in;

uniform mat4 view;
uniform mat4 proj;

void main()
{
    // gl_Position = proj * view * vec4(pos_vs_in, 1);
    pos_fs_in = pos_vs_in;
    gl_Position = vec4(pos_vs_in, 1);
}