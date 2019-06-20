
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
    -r 10 \
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


+ low priority
    + optimized forward kinematics in shaders 
    + depth counter algorithm for self intersection volume computation




## meeting 

#### before March 22

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


#### March 22

+ faster stress computation 
    + GPU accelerated Jacobi solver
    + voxel representation of mesh 
        + no matrix assembly since its on a grid
        + solving linear system `Ax=b` with iterative solver on CPU first then GPU 
            + shader (cross platform) vs. cuda
    + reference 2d jacobi solver for poisson equation ... on CPU/GPU
+ ui
    + find closest quaternion to match screen space drag, or
    + github repo given ImGuizmo
+ engineering
    + cmake release/debug mode 


#### May 23 


+ what have we done so far
    + goal
        + a design tool to allow artistics to deform figures by their skeleton
        + be able to visualize yield stress on surface as they make edit
        + explore deformations with good properties with global optimization
    + global optimization
        + reduced space (skeleton) enabling possibility of global optimization 
        + objective function capturing requirements:
            + minimize supporting material used
            + able to stand (center of mass inside support polygon)
            + will not break (yield criterion > material-specific yield stress)
            + local distortion (arap energy)
    + stress visualization (need real time)
        + explored
            + iterative methods like jacobi, SOR (can control number of iterations)
            + hexahedron element over axis-aligned grids (element stiffness K constant)
        + later
            + multigrid on cuda

#### June 4
+ forget about the continuous derivation for shape gradient
    + _variational surface cutting_ has a different formulation from our problem
+ discrete setting for solving the PDE-contrained optimization problem
    + convert the constraints into _K(d)u=f(d)_ using FEM, where d is the design parameter (in our case the vertex positions)
    + take derivative of K and f with respect to d
    + look into _automatic differentiation_
+ refer to Bernhard Thomaszewski's papers to get a better sense regarding how the method works
+ the ultimate goal doesn't change
    + object function: min E_{ARAP}(X_{input mesh},X(T))+E_{stress}(X_{input mesh},X(T))
+ next step
    + start with a simple 2D cantilever example
        + linear FEM stiff material
        + nonlinear FEM soft material
        + treat X as a function of T (apply LBS)

#### June 6
+ the deformed volumetric mesh might be of bad quality due to LBS
    + not suitable to do FEM based on this
    + need to confirm in both 2D and 3D
+ use auto diff to compute the partial derivative of K with respect to T
+ in each frame voxelize the deformed tet mesh
+ do trilinear interpolation over the bbw weights field for each grid point
    + to get the variable back to T


#### June 20
+ instead of doing voxelization, use fixed grid
    + look at ChainQueen: A Real-Time Differentiable Physical Simulator for Soft Robotics
    + the approach that is used here (tet <-> hex) is similar to what is commonly used in CFD
    + ask Michael Tao for details
+ dK(p)/dp * u has equivalent result as d(K(p)*u)/dp
    + u is simply a few numbers u = Kf
    + the difference is the complexity -> the sparsity of K
+ stan is good

+ questions
    + continuous optimization (whether or not use gradient ...)
        + consider arap energy direction ?
        + `min_\Omega ( \min_u \int_\Omega E(u,X) dx))`
            + `\Omega` is the domain
        + https://nmwsharp.com/media/cea_tutorial.pdf
        + hard constraints (make it stand)
            + global optimization use hard constaints
            + then use gradient for different initial points.
        + do not penalize small values of stress 
            + can design functions that is continuous 
                + (0 infeasible, grows very quickly for feasible)
        + need to consider the feasible set ... (should be good)
        + todo
            + do continous optimization on arap+fracture energy 
            + how do you derive gradients
                + derivation of gradient of fracture `u` with respect to `X`
                    + https://nmwsharp.com/media/cea_tutorial.pdf
            + do 2d shape optimization
                + on simple coarse domain, the rod that sticks out of wall
                + expect points close to wall expand
        + hybrid global optimization methods
            + design gallery: probably better to use global methods
        + what is the thing we want to compute gradient of ?
            + in the want to compute gradient with respect to euler angle
            + `X(\theta)` where `X` is the surface
    + is it a good idea to go with multigrid ? 
        + pro: O(n) complexity
    + can we do weekly meetings ?
    + interpolation problem is fixed
        + do a screenshot to show it's done

+ thoughts
    + at first identify your problem rather than jumping into the problem i.e. what the problem it is
    + look into the closest work, try to understand their work and make comparisons