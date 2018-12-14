
### Install

```
git clone --recursive git@github.com:tt6746690/fast_support_reduction.git

# profiling: minitrace + chrome tracing 
#       https://github.com/hrydgard/minitrace
#       https://www.chromium.org/developers/how-tos/trace-event-profiling-tool

mkdir build && cd build

# debugging, or
cmake ..
# 03! or
cmake .. -DWITH_DEBUG=OFF
# visualize overhangs
cmake .. -DWITH_VISUALIZE=ON

# run 
# build/support_reduction 
#       filename \
#       n_fixed_bones \
#       pso_iters \
#       pos_population \
#       alpha_max \
#       rotation_angle \
#       c_arap \
#       c_overhang \
#       c_intersect \
#       rotate_model
build/support_reduction bb-bunny 5 1 1 50 90 20 40 0.5 0


# debugging
lldb build/support_reduction
process launch -- hand 1 1 1 45 20 100 1 10 0
```

### Layout

```
.
├── CMakeLists.txt
├── README.md
├── abstract                    // extended abstract
├── cmake
├── data                        // source and destination directory for mesh, weights, ...
├── libigl
├── makefile
├── matlab
├── papers
├── src                         // source code
├── bbw.cpp                     // for generating bounded biharmonic weights and remeshing
├── support_reduction.cpp       // run program
└── visualize.cpp               // visualize self-interesection
```
