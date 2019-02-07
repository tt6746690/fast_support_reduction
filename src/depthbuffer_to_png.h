#pragma once

#include <string>

// outputs .png file containing depth texture of currently bound framebuffer object
bool depthbuffer_to_png(
    const std::string png_file,
    const int width,
    const int height);
