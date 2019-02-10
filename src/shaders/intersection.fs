#version 410 core

in  vec2 tex_coord;
out vec4 color;

uniform sampler2D cur_depth_texture;
uniform sampler2D prev_depth_texture;
uniform sampler2D prevprev_depth_texture;
uniform sampler2D cur_color_texture;
uniform sampler2D prev_color_texture;
uniform sampler2D prevprev_color_texture;
uniform sampler2D acc_color_texture;

void main()
{
    float cur_depth = texture(cur_depth_texture, tex_coord).r;
    vec4 t_acc_color = texture(acc_color_texture, tex_coord);

    if (cur_depth == 1) {
        color = vec4(t_acc_color.rgb, 1.);
    } else {
        vec4 t_cur_color = texture(cur_color_texture, tex_coord);
        vec4 t_prev_color = texture(prev_color_texture, tex_coord);
        vec4 t_prevprev_color = texture(prevprev_color_texture, tex_coord);
        bool cur_frontfacing = t_cur_color.r == 1;
        bool prev_frontfacing = t_prev_color.r == 1;
        bool prevprev_frontfacing = t_prevprev_color.r == 1;

        // self-intersection criterion
        //      - fragment is not on far plane
        //      - current normal changed dir, given 2 prev layer has same normal dir
        float selfintersect_depth;
        if ((prev_frontfacing && prevprev_frontfacing && !cur_frontfacing) ||
            (!prev_frontfacing && !prevprev_frontfacing && cur_frontfacing))
        {
            float prev_depth = texture(prev_depth_texture, tex_coord).r;
            selfintersect_depth = cur_depth - prev_depth;
        } else {
            selfintersect_depth = 0.;
        }

        color = vec4(t_acc_color.rgb+selfintersect_depth, 1.);
    }
}