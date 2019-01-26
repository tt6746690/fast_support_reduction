
### Install

```
git clone --recursive git@github.com:tt6746690/fast_support_reduction.git

# profiling: minitrace + chrome tracing 
#       https://github.com/hrydgard/minitrace
#       https://www.chromium.org/developers/how-tos/trace-event-profiling-tool

mkdir build && cd build

# brew install ninja
cmake -GNinja ..

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



#### Readings


+ coefficient
    + tune to be fixed 
    + account for volume / surface
    + one way to test things work: scale mesh by 100x and see result is invariant

+ support generation
    + meshmixer generates support 
    + varied for different software / other scenario

+ the cool part 
    + joint based character deformation 

+ how to push forward to a publication 
    + make things fast
    + dont worry about support reduction 
        + varied by software used 
        + probably material/context specific algorithm/technique for support reduction and so a general black box is not easy to be motivated.
    + make this about posing sculptures: take into account physics of material
        + print out cement ... need to make sure it does not fracture
    + impl detail
        + remove overhang
        + add self-balancing (center of mesh project into convex hull)
            + surface intergrall to determine mass center volumetrically 
        + add load+stress
            + linear elasticity
            + stress is a linear function of displacement
            + `KV = KMT` where `V=MT`
                + `K` given beforehand
                + `KM` is `6n x m` can be put in vertex shader?
            + need to account for things not covered by the first rig
                + another rig?
            + need to account for fracture in interior?
                + enough to render stress on boundary ?
                + need to dig with this later
        + interface for 
            + hints for intersection, position of stress
            + then option for global optimization 

+ todo  
    + understand physics of stress field
        + ask sarah on physics based stuff.
    + rendering based speed up