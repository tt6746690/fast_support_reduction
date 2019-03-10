
## Learning Resources


+ https://cseweb.ucsd.edu/classes/sp16/cse169-a/readings/3-Skin.html
    + background on skinning
    + simple skinning
        + mesh as a single continuous mesh
        + every vertex attached to 1 joint, 
        + final transformation is the transformation of the joint
    + smooth skinning (linear blending skinning)
        + each vertex in mesh attached to >=1 joints, with different weights, 
        + final transformation for each vertex is weighted average of transformation for each of its assigned joint

+ [cs426_ch3_skinning](cs426_ch3_skinning.pdf)
    + skinning
        + deform skin to match motion of skeleton
    + linear blend skinning / skeleton subspace deformation (SSD)
        + assign weights for every vertex on the skin mesh to underlying bones
            + weights sum to 1
        + rest space
            + world space where we have placed skin mesh at rest and the skeleton at rest inside it, ... 
        + local bone coordinate
            + 1 vertex relative to different bones has different coordinate
        + world space
            + similar to rest space, for which `T{W<-k}` is a transformation, as a function of time
        + mapping between the two spaces
            + i.e. transformations
        + skin vertex
            + weighted average of where all the bones think it should be
                + `p_rest = \sum_{k bones} w * T{R<-k} * p_local`
                + local position transformed to rest position for each bone, then weighted average the positions
    + problem with linear blending
        + rotate joint -> shrink volumne
        + artifact with too much rotation

## papers

+ [2011_bounded_biharmonic_weights_for_real_time_deformation](2011_bounded_biharmonic_weights_for_real_time_deformation.pdf)
    + abstract
        + developed linear blending skinning weights automatically for different handle/control types (points, bones, and cages)
        + weights minimizes laplacian energy subject to bound constraints, with goal of
            + smooth/intuitive deformation
            + high-res meshes
            + smooth deformation near controls
            + local support, influence of control exerts on.
        + real-time computation
    + problem formulation
        + controls
            + cages: a collection of simplices, requiring that simplices transform linearly as the cage vertices are translated.
            + joint shared by connecting bones
        + linear blending
            + $p' = \sum_{j=1:m} = w_j(p) T_j p$
        + find $w_j(p)$ by
            + minimize squared laplacian of weights over the volumne
            + subject to 
                + weights on points on handles equals 1
                + cage weights linear
                + sum of weights to 1
                + _bounded_ weights between 0 and 1
                    + ensures no local maxima
                    + ensures non-negativity of weights
            + yield some nice properties...
            + a constrained quadratic programming problem
    + shape preservation 
        + areas specified by user s.t. al weight functions are constant.... such that shape is preserved  
    + discretization
        + mesh domain Sigma to be compatible with handles and vertices of S
            + used Triangle (2D) and TetGen (3D)
            + http://wias-berlin.de/software/tetgen/files/tetgen-manual.pdf
        + FEM for discretizing laplacian 
        + sample vertices on skeleton bones / cage faces,
    + optimization
        + quadratic programming with MOSEK
        + lifting partition of unit parity constraint parallelizes weight computation for each handle, faster!
    + specify handle transformations
        + use translation to specify rotation
    + experiments and results
        + weights free of spurious local maxima
        + points, skeleton, and cage can be used in conjunction with each other
        + generalize to 3D
        + partial cage (3D caging is tedious) to control parts of object without it having to surround it fully
        + quite fast
    + questions
        + why use tetrahedron for 3D mesh? and not triangles
            + because domain of `w_j` is the volume
            + needs tet mesh for 3D case, since need to sample discretely over domain of `w_j` to compute integral inside the objective function
        + a bit confused about how the _linear_ constraint is enforced
            + not relevant in our project, since caging not a viable handle type


+ [2012_fast_automatic_skinning_transformations](2012_fast_automatic_skinning_transformations.pdf)
    + goal
        + method for deforming shapes as naturally as possible when a subset of degrees of freedom are specified
        + fast energy minimization
            + objective w.r.t. handle transformation only
            + use skinning weights to determine parts of shape that are likely to be transformed similarly and simplify deformation energy
    + related work
        + inverse kinematics
            + infers missing dof. on skeleton of a character
        + shape deformation (2 streams)
            + _variational_ how a shape should deform with arbitrary fixed handles in space
                + this paper uses ARAP, an energy measuring a form of elastic shape distortion
            + _closed-form_ how to define influence of each handle at the handles are proparated to each point on the shape by local weighted combination.
                + e.x. bouned biharmonic weights 
                + fast, parallelizable
                + but have to specify full affine transformation per handle!
            + _reduced model_ a combination of variational and closed-form 
                + minimize some shape energy on small number of degrees of freedom (instead of the entire shape) in a simpler (faster) deformation subspace
    + method
        + deform handle vertices `h' = Th` by given transformation `T_j` for each handle
        + compute deformed shape based on constraints on the handle
            + LBS deformation of mesh `M` vertices given by `V' = MT`,
                + `M` matrix combining rest-pose vertex position and vertex weights
                + `T` stacks transposed transformation matrices for each handle `T_j`
    + automatic degrees of freedom
        + degrees of freedom for handles
            + unconstrained -> shape-aware inverse kinematics
            + `T_j` translations only -> unnatural sheared deformations
        + solution
            + optimize remaining degrees of freedom of the handle transformations that user did not specify.
            + constrain
                + transformation that is fixed
                + location of transformed points, i.e. fixed
            + minimize 
                + ARAP energy between rest-pose shape `M` and deformed shape `M'`
                + energy w.r.t.
                    + handle transformation `T`
                    + local rotations `R` 
    + rotation clusters
        + motivation
            + motions of neighboring vertices highly correlated, so find clustered vertices and let them undergo similar deformations
        + idea
            + cluster vertices based on euclidean distance in _weight space_
                + each vertex `v_i` assigned a vector of skinning weights,
            + k-means clustering to find grouping
    + additional weight function
        + idea
            + create a set of abstract handles whose transformation entirely determined by optimization. 
            + if all handles constrained, then FAST reduces to LBS, not ideal!
        + step
            + smooth isotropic cubic B-spline basis function in weight space
    + question
        + why vertex 4D, 1 in the end
        + difference between closed form deformation with paper's method. In both cases T_j needs to be specified beforehand
            + advantage of the method is there is automatic inference of some degrees of freedoms
        + local rotation not known in our project?
            + the search space is then both `T` and `R` where `R` is clustered to groups
