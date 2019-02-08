#include "glad/glad.h"

#include "depthbuffer_to_png.h"
#include <igl_stb_image.h>
#include <iostream>

bool depthbuffer_to_png(
    const std::string png_file,
    const int width,
    const int height)
{
    unsigned char* D = new unsigned char[width*height];
    glReadPixels(0, 0, width, height, GL_DEPTH_COMPONENT, GL_UNSIGNED_BYTE, D);
    unsigned char* I = new unsigned char[4*width*height];
    for (int i = 0; i < width; ++i) {
        for (int j = 0; j < height; ++j) {
            for (int k = 0; k < 3; ++k) {
                I[4*(j*width+i)+k] = D[j*width+i];
            }
            I[4*(j*width+i)+3] = 255;
        }
    }

    bool ret = igl::stbi_write_png(png_file.c_str(), width, height, 4, I, 4*width*sizeof(unsigned char));
    delete [] D;
    delete [] I;
    return ret;
}