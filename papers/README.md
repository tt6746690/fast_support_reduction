

# Unclassified


#### Reducing support for 3D printing

+ [2014_clever_support_efficient_support_structure_generation_for_digital_fabrication](2014_clever_support_efficient_support_structure_generation_for_digital_fabrication.pdf)
    + goal
        + reduction of support structures required by 3D printers
    + steps
        + orient the input model
        + detect overhang points
            + point, face, edge overhangs
            + face: ... if printing direction and tangent of triangle exceed some criticl angle
        + tree-like support2012
            + uniform sampling on mesh for support

+ [2013_make_it_stand_balancing_shapes_for_3D_fabrication](2013_make_it_stand_balancing_shapes_for_3D_fabrication.pdf)
    + goal
        + make 3D printing object stand by carving and deforming
    + steps
        + input: surface mesh, contact points, orientation
        + improve stability by redistributing weights for both interior and surface by
            + carving inside volume
                + interior as voxel grid (0-empty, 1-filled)
                + translation, scaling, rotation of volumn
            + deforming surface
        + while minimizing deviation from intended shape
    

+ [2015_support_slimming_for_single_material_based_additive_manufacturing](2015_support_slimming_for_single_material_based_additive_manufacturing.pdf)
    + goal 
        + deform model `M` into self-supporting state `AM` to reduce support
    + steps
        + cage model `M` inside a volumetric mesh `T`
            + courser `T` faster convergence, and 
            + preserves detail on `M`
        + Find reorientation of _facing-down_ surfaces for
            + global minimization of rigidity energy (shape variation)
            + local minimization of rotation
        + global/local optimization to 
            + reduce number of _face-down_ surfaces
            + reduce distortion (similar to ARAP)
    + take-away
        + how to formulate a ARAP-like energy function
    

#### Intersection detection 

+ [2017_generalized_matryoshka_computational_design_of_nesting_objects](2017_generalized_matryoshka_computational_design_of_nesting_objects.pdf)
    + goal  
        + optimal self-nesting
            + find largest scale replica of object inside the object w.t.
            + no penetration (bounding)
            + can cut outer in two and take out the inner object without collision
        + parallelisation with gpu
    + feasibility analysis
        + fast method to determine
            + nesting: transformation T s.t. T(A) in A
            + w.r.t. to cut plane P: exists removal trajectory vector a+, a- that every point in boundary of A above/below P has a clear line of sighe intersecting P
        + [order_independent_transparency](order_independent_transparency.pdf)
            + more on depth peeling 
        + uses depth peeling and some logic statement to determine feasibility
    + scale search
        + fix P and a+ , a-, assume convexity of shape and use binary search for lower and upper bound on scaling factor `s` such that it is the largest `s` passing feasibility analysis
    + global optimization
        + all parameter free to be optimized, i.e 
            + rotation R, 
            + scaling s, 
            + about a displaced centroid c, 
            + cut plane P
            + removal tragectory vector a+, a-
        + minimize s, s.t. configuration passes feasibility test (non-linear, non-convex, but fast method)
        + optimization: particle swarm
        + problem
            + spent too much time sampling small values of s, not optimal
    + acceleration


#### Casting 

+ [2018_interactive_design_of_castable_shapes_using_two_piece_rigid_molds](2018_interactive_design_of_castable_shapes_using_two_piece_rigid_molds.pdf)
    + abstract
        + tool to generate two-piece rigid modls separated by a plane cut.
        + castabililty energy optimized with gradient-based methods with elastoplastic deformation based on ARAP
    + castability
        + component of a mold, called _state_
            + cast mesh 
                + `(V,F)`
            + cut plane 
                + 4-vector `p`, first 3 plane normal, 4th an offset of plane from origin
            + removal directions
                + `d1`,`d2`  for each side of the plane
        + goal
            + arrive at a _castable_ state
                + cut along cut plane
                + cast object can be removed from mold with collison-free translation, or _removability condition_
                    + `N_f \dot d >= \tau` for all `f \in F`
    + castability energy
        + ARAP energy with local-global optimization
        + enforce castability
            + during rotational step of ARAP optimization, check if removability condition is fulfilled, if not, rotate tetrahedron around `dxN` that this face belongs until removability condition is satisfied
    + steps
        + pick initial _state_
        + until shape castable, repeat
            + find optimal removal directions by minimizing castability energy
            + run ARAP until convergence
        + post-processing minimizing castability energy, taken into account tolerance levels


#### Signed distance field

+ [2013_robust_inside_outside_segmentation_using_generalized_winding_numbers](2013_robust_inside_outside_segmentation_using_generalized_winding_numbers.pdf)
    + algorithm for 
        + boundary/surface representation (triangle mesh) -> volumetric representation (tetrahedral mesh, i.e. voxels)
        + i.e. discretization of input's inner volume
        + deals with geometric/topological artifacts
            + self-intersection 
            + open boundaries 
            + non-manifold edges
    + Delaunay triangulation (CDT)
        + for a given set P of discrete points in a plane is a triangulation DT(P) such that no point in P is inside the circumcircle of any triangle in DT(P).
        + then segment to inside and outside volume
    + comparison
        + winding number
            + signed length of the projection of a curve onto a circle at a given point divided by 2pi
                + outside, projection cancels out, -> 0
                + inside, projection = 1
            + sharp jump at the surface, smooth elsewhere
            + a piecewise-constant segmentation of space when input is perfect (i.e. watertight)
            + if not perfect, then function well=behaved and will guide downstream E-minimization algorithms
        + signed distance field
            + do not encode segmentation confidence! smooth when crossing the surface
    + steps
        + construct inside-outside confidence function which generate the winding number
        + evaluate integral average of this function at each element in a CDT containing (V,F)
        + select a subset \Epsilon of CDF via graphcut energy optimization enforcing facet interpolation and manifoldness
    + winding number
        + is number of full revolution an observer at p tracking a moving point along C took
        + 2D
            + signed length of projection of a curve C onto the unit circle around p divided by 2pi
                + 0 -> outside
                + 1 -> inside
            + full ccw rotation  ->  w(p) = 1
            + full cw rotation -> w(p) = -1
        + 3D genearlization 
            + solid angle is the signed surface area of projection of a surface S to a point in R^3
                + angle -> solid angle
            + w(p)=  solid_angle(p) / 4pi
        + immediate discretiztion, i.e. 
            + if C is piecewise linear
            + if S is triangulated piecewise linear surface
    + open, nonmanifold, ...
        + winding number is harmonic except on C/S, implying C^{infty} smoothness and minimal oscillation
        + jump +-1 cross boundary is a confidence measure, 
    + hierarchical evaluation
        + computing w(p): simply sum the contribution of each triangle in a mesh, easily parallezable
    + segmentation & energy minimization with graphcut
        + have w(p), select a subset of CDT of convex hull of (V, F)
    + future direction 
        + winding nubmer rely on orientation of input facets, triangle soups / or with erroneous orientation need further processfing
    + conclusion    
        + respects self-intersection and correctly identifies regions of overlap in presense of artifacts such as holes



+ [2016_thingi10k_a_dataset_of_10000_3D_printing_models](2016_thingi10k_a_dataset_of_10000_3D_printing_models.pdf)
    + 10K good quality 3D printing model with annotation and is queriable

+ [2017_pointnet_deep_learning_on_point_sets_for_3D_classification_and_segmentation](2017_pointnet_deep_learning_on_point_sets_for_3D_classification_and_segmentation.pdf)

+ [2017_voxelnet_end_to_end_learning_for_point_cloud_based_3D_object_detection](2017_voxelnet_end_to_end_learning_for_point_cloud_based_3D_object_detection.pdf)
    + learn discriminative 3D represetation from point clouds and predicts accurate 3D bounding box


+ [2018_fast_winding_numbers_for_soups_and_clouds](2018_fast_winding_numbers_for_soups_and_clouds.pdf)
    + abstract
        + generalize winding number to point clouds
            + i.e. determine if a point is inside or outside
        + improve runtime speed for triangle soups and point clouds
            + tree based algorithm to reduce complexity, O(log m) amortized
    + application 
        + with Thingi10k + winding number -> signed voxel grid for GAN training and shape generation
    + what is bad about 2013 paper
        + direct summation of contribution of each triangle mesh is too slow
            + sublinear complexity with divide and conquer
            + GOOD in general, but needs pre-computation
            + also in case of triangle clouds, degenerates into slow direct sum
    + oriented point clouds
        + list of points P + normal vector N or m oriented triangles
        + outputs a function w, which computes winding number of any points
        + generalization 
            + mesh: solid angle computed via inverse tangent function

