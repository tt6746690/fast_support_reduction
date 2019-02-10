#version 410 core

out vec4 color;

uniform float width;
uniform float height;

uniform sampler2D prev_depth_texture;
uniform sampler2D prevprev_depth_texture;
uniform sampler2D prev_color_texture;
uniform sampler2D prevprev_color_texture;
uniform sampler2D acc_color_texture;

void main()
{
    vec2  tex_coord = vec2(gl_FragCoord.x/width, gl_FragCoord.y/height);
    float max_depth = texture(prev_depth_texture, tex_coord).r;
    if (gl_FragCoord.z <= max_depth) {
        discard;
    }


    // vec2 tex_coord = vec2(gl_FragCoord.x/width, gl_FragCoord.y/height);
    vec4 t_prev_color = texture(prev_color_texture, tex_coord);
    vec4 t_prevprev_color = texture(prevprev_color_texture, tex_coord);
    bool prev_frontfacing = t_prev_color.r == 1;
    bool prevprev_frontfacing = t_prevprev_color.r == 1;

    // self-intersection criterion
    //      - fragment is not on far plane
    //      - current normal changed dir, given 2 prev layer has same normal dir
    float selfintersect_depth;
    if (gl_FragCoord.z != 1 && (
        (prev_frontfacing && prevprev_frontfacing && !gl_FrontFacing) ||
        (!prev_frontfacing && !prevprev_frontfacing && gl_FrontFacing)
    ))
    {
        vec4 t_prev_depth = texture(prev_depth_texture, tex_coord);
        float prev_depth = t_prev_depth.r;
        selfintersect_depth = abs(gl_FragCoord.z - prev_depth);
    } else {
        selfintersect_depth = 0.;
    }

    vec4 t_acc_color = texture(acc_color_texture, tex_coord);
    color = vec4(t_acc_color.rgb+selfintersect_depth, 1.);
}
