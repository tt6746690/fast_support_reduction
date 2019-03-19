
### Install

```
git clone --recursive git@github.com:tt6746690/fast_support_reduction.git

# profiling: minitrace + chrome tracing 
#       https://github.com/hrydgard/minitrace
#       https://www.chromium.org/developers/how-tos/trace-event-profiling-tool

# dependencies
brew install cgal

mkdir build && cd build

# brew install ninja
cmake -GNinja ..

# openmp
export OMP_NUM_THREADS=4

# mesh file generator: tetrahedralize the input surface mesh and generate bbw weights for each corresponding tet
./mesh_file_generator dinosaur
input files: ../dinosaur.obj ../dinosaur.tgf
output file: ../data/dinosaur.mesh ../data/dinosaur.dmat


# run
./support_reduction \
    -d ../data/ \
    -f bb-bunny \
    -b 5 \
    -i 1 \
    -p 1 \
    -r 50 \
    -a 90 \
    -c 20 \
    -e 1000

# debugging
lldb support_reduction
process launch -- -d ../data/ -f bb-bunny -b 5 -i 1 -p 1 -r 50 -a 90 -c 20

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

#### TODO

+ euler angle
+ optimized forward kinematics
+ depth counter algorithm for self intersection volume computation



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