#version 410 core

in  vec4 pos_fs_in;
out vec4 color;

void main()
{   
    color = vec4(vec3(1, 0.5, 0.1)*pos_fs_in.xyz, 1);
}