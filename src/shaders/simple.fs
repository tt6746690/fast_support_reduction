#version 410 core

in vec4 pos_fs_in;
out vec3 color;
void main()
{
  // Set color to screen position to show something
  // color = 0.5+0.5*pos_fs_in.xyz;
  color = vec3(1,1,1);
}