#version 410 core

in  vec3 pos_vs_in;
out vec2 tex_coord_fs_in;

uniform mat4 mvp;

void main()
{
    tex_coord_fs_in = vec2(0.5*(pos_vs_in.x+1), 0.5*(pos_vs_in.y+1));
    gl_Position = mvp * vec4(pos_vs_in, 1);
}