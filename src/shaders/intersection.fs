#version 410 core

out vec4 color;

uniform float width;
uniform float height;
uniform int which_pass;

uniform sampler2D prev_depth_texture;
uniform sampler2D prevprev_depth_texture;


void main()
{
    color = vec4(1,1,1,1);
    color *= gl_FragCoord.z;
    if (!(which_pass == 0)) {
        vec2 tex_coord = vec2(gl_FragCoord.x/width, gl_FragCoord.y/height);
        float max_depth = texture(prev_depth_texture, tex_coord).r;
        if (gl_FragCoord.z <= max_depth) {
            discard;
        }
    }
}
