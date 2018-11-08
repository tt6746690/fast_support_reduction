





#### Questions


+ question 
    + how does 3D -> 2D rendering works? depth peeling
    + displaced centroid?

+ idea
    + skeleton based deformation for support for 3D printing
+ bounded biharmonic weights for real-time deformation
    + given skeleton, gneerate weights for each vertex
+ fast automatic skinning transofmration 
    + ARAP skeleton

+ todo
    + 2D prototype, 
        + use http://libigl.github.io/libigl/python-bindings/
        + generalize overhang energy term to skeleton...
    + libigl examples


#### API 


+ https://github.com/libigl/libigl/blob/master/include/igl/directed_edge_parents.h
+ https://github.com/libigl/libigl/blob/master/include/igl/harmonic.h
+ https://github.com/libigl/libigl/blob/master/include/igl/bbw.h
+ https://github.com/libigl/libigl/blob/master/include/igl/lbs_matrix.h


+ formats
    + `dmat`: human readable eigen matrix
    + `tfg`: handles for deformation
    + `off`: mesh, i.e. `(V,F)`
    + `obj`: mesh from ascii obbj file, vertex, normal, texture, 
    + `mesh`: tetrahedron mesh  loads `V,T,F` 
        + vertices `(,3)` (7234)
        + tets `(,4)` (29998)
        + faces `(,3)` (9556)


#### TODO


+ ask alec to gives code on mashoysha
+ bounded harmonic weights tut403
    + `boundary_condition` not correspond to paper
    + does not work, so need to figure out how to do this
    + ill try to make bbw work in 2D, otherwise well move to 3d


+ closed form solution 
    + derivation for overlap and support reduction


