#version 410 core

in  vec3 pos_vs_in;
out vec2 tex_coord;

void main()
{
    gl_Position = vec4(pos_vs_in, 1);
    tex_coord = vec2(0.5*(pos_vs_in.x+1), 0.5*(pos_vs_in.y+1));
}