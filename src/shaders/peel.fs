#version 410 core

out vec4 color;

uniform float width;
uniform float height;
uniform int which_pass;

uniform sampler2D prev_depth_texture;


// encode depth `v` (in [0,1]) and frontfacing (a bool) to `[0,1]`
// [0,0.4]        backfacing depth
// [0.5,0.9]      frontfacing depth
float encode(float v, bool frontfacing) {
    float depth;
    if (frontfacing) {
        depth = gl_FragCoord.z/2.5 + 0.5;
    } else {
        depth = gl_FragCoord.z/2.5;
    }
    return depth;
}

// decoding `encoded_depth` to depth and frontfacing
// 
// if (encoded_depth >= 0.5) {
//     depth = 2.5*(encoded_depth-0.5);
//     frontfacing = true;
// } else {
//     depth = 2.5*encoded_depth;
//     frontfacing = false;
// }

void main()
{
    if (!(which_pass == 0)) {
        vec2  tex_coord = vec2(gl_FragCoord.x/width, gl_FragCoord.y/height);
        float encoded_depth = texture(prev_depth_texture, tex_coord).r;
        float max_depth;
        if (encoded_depth >= 0.5) {
            max_depth = 2.5*(encoded_depth-0.5);
        } else {
            max_depth = 2.5*encoded_depth;
        }
        if (gl_FragCoord.z <= max_depth) {
            discard;
        }
    }

    if (gl_FrontFacing) {
        color = vec4(1,0,0,1);
    } else {
        color = vec4(0,1,0,1);
    }
    color *= gl_FragCoord.z;

    gl_FragDepth = encode(gl_FragCoord.z, gl_FrontFacing);
}