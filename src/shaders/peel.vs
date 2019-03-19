#version 410 core
#define MAX_BONES 15 // have to explicitly specify the size of array
layout(location = 0) in vec3 pos_vs_in;
layout(location = 1) in float[MAX_BONES] W;

uniform mat4 T[MAX_BONES];
uniform mat4 model_view_proj;
uniform int num_bones;

void main()
{
    // per-vertex linear blend skinning
    vec4 pos = vec4(0, 0, 0, 0);
    for (int i = 0; i < num_bones; i++) {
        pos = pos + W[i] * T[i] * vec4(pos_vs_in, 1);
    }
    gl_Position = model_view_proj * pos;
}