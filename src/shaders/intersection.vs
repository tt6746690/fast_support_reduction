#version 410 core

layout(location = 0) in  vec3 pos_vs_in;

uniform mat4 model_view_proj;

void main()
{
    gl_Position = model_view_proj * vec4(pos_vs_in, 1);
}