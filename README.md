
### Install

```
git clone --recursive git@github.com:tt6746690/fast_support_reduction.git

# profiling: minitrace + chrome tracing 
#       https://github.com/hrydgard/minitrace
#       https://www.chromium.org/developers/how-tos/trace-event-profiling-tool

mkdir build && cd build

# brew install ninja
cmake -GNinja ..

# openmp
export OMP_NUM_THREADS=4

# run (Clara: remember flags!)
./build/support_reduction \
--data_dir data/ \
--filename bb-bunny \
--n_fixed_bones 5 \
--pso_iters 5 \
--pso_population 20 \
--rotation_angle 50 \
--c_arap 90 \
--c_overhang 20 \
--c_intersect 40

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


#### self-intersection with shader


+ GPU-accelerated evaluation of self-intersection volume
    + problem
        + given a water-tight mesh and projection direction
        + framebuffer width and height, for controlling how accurate the computation is
        + return self-intersecting volume
    + preparation
        + find bounding box of the mesh
        + determine orthographic projection around the bounding box of the mesh
    Peel off front-most fragment of mesh along an orthographic viewing direction
    aligned with the inverse of printing direction (i.e. projection direction).
    The near/far plane of orthographic projection are faces of the bounding box of the mesh.
    For each fragment, record 

#### TODO for eris

+ compute percentage volume
    + self-intersection volume: sum of color texture red channel after while loop.
    + total volume: 255*width*height
+ compute absolute volume
    + percentage volume * (ortho viewing box 2^3)
+ ortho view from normalized device coordinate -> a bounding sphere of the mesh 
    + end goal: avoid a copy/resize for mesh 
    + check matroyshka
+ implement linear blend skinning in shader ...
    + weights preloaded as
        + (best) in vertex buffer
        + (ok) as uniform
    + pass bones transformation matrix as uniform 
    + do lbs in vertex shader ...
+ hook selfintersctionvolume to rest of pipeline
    + check runtime/speedup


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