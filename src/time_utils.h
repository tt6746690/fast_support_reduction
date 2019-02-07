#pragma once

// Return current epoch time in seconds
double get_seconds();

// Sleep to reach desired fps 
//      where `tic` is time at start of the render loopo
void sleep_by_fps(int fps, double tic);