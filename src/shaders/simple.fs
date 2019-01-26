#version 410 core

in  vec4 pos_fs_in;
out vec4 color;

void main()
{
    // Set color to screen position to show something
    color = vec4(0.5,0.5,0.5,1);

} 