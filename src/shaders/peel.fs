#version 410 core

out vec4 color;

uniform float width;
uniform float height;
uniform int which_pass;

uniform sampler2D prev_depth_texture;

void main()
{
    if (!(which_pass == 0)) {
        vec2  tex_coord = vec2(gl_FragCoord.x/width, gl_FragCoord.y/height);
        float max_depth = texture(prev_depth_texture, tex_coord).r;
        if (gl_FragCoord.z <= max_depth) {
            discard;
        }
    }

    // use color to represent normal direction of fragment
    //      frontfacing:        color.r == 1
    //      backfacing:         color.r == 0  (color.g == 1 for visualization's sake)
    if (gl_FrontFacing) {
        color = vec4(1,0,0,1);
    } else {
        color = vec4(0,1,0,1);
    }

}