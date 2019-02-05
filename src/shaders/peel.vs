#version 410 core

layout(location = 0) in  vec3 pos_vs_in;

out vec4 pos_fs_in;

uniform mat4 model_view_proj;

void main()
{
    pos_fs_in = model_view_proj * vec4(pos_vs_in, 1);
    gl_Position = pos_fs_in;
}