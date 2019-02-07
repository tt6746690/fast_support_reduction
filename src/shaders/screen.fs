#version 410 core

// in  vec4 pos_fs_in;
in  vec2 tex_coord_fs_in;
out vec4 color;

uniform sampler2D depth_texture;

void main()
{
    float d = texture(depth_texture, tex_coord_fs_in).s;
    color = vec4(d, d, d, 1);
}