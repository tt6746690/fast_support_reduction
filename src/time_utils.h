#pragma once

// Return current epoch time in seconds
double get_seconds();

// Sleep to reach desired fps 
//      where `tic` is time at start of the render loopo
void sleep_by_fps(int fps, double tic);


// Implementation

#include <chrono>
#include <thread>

double get_seconds() {
    return std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()
    ).count();
}

void sleep_by_fps(int fps, double tic) {
    double duration = 1000000.*(get_seconds()-tic); // In microseconds
    const double min_duration = 1000000./60.;
    if(duration<min_duration) {
        std::this_thread::sleep_for(
            std::chrono::microseconds((int)(min_duration-duration)));
    }
}