#ifndef __REDUCE_SUPPORT_H__
#define __REDUCE_SUPPORT_H__
#include <Eigen/Core>
#include <Eigen/Sparse>


template <typename T>
class ReduceSupportConfig{
public:
    ReduceSupportConfig()
        :   is3d(true),
            alpha_max(0.25*M_PI), 
            dp(0.,1.,0.),
            rotation_angle(0.25*M_PI),
            pso_iters(1),
            pso_population(1),
            c_arap(1),
            c_overhang(1),
            c_intersect(1),
            unsafe(),
            display(false)
        {}

    // ctor conversion
    template <typename U>
    ReduceSupportConfig(const ReduceSupportConfig<U>& that) {
        this->is3d = that.is3d;
        this->alpha_max = (T) that.alpha_max;
        this->dp = that.dp.template cast<T>();
        this->rotation_angle = (T) that.rotation_angle;
        this->pso_iters = that.pso_iters;
        this->pso_population = that.pso_population;
        this->c_arap = (T) that.c_arap;
        this->c_overhang = (T) that.c_overhang;
        this->c_intersect = (T) that.c_intersect;
        this->display = that.display;
        this->unsafe = that.unsafe;
    }

public:

    // dimension of input mesh 
    bool is3d;

    //  maximal self-supporting angle
    T alpha_max;
    //  normalized printing direction
    Eigen::Matrix<T, 3, 1> dp;

    //  maximum range of rotation for each bone
    T rotation_angle;
    //  number of iterations for `pso`
    int pso_iters;
    //  size of particle swarm `pso`
    int pso_population;

    // coefficient to different energy terms
    T c_arap;
    T c_overhang;
    T c_intersect;

    // saved values for visualization
    Eigen::MatrixXi unsafe;

    bool display;
};



// Given mesh (V, F), find minimizer of energy using particle swarm optimization
//      
//      E = E_{arap} + E_{overhang} + E_{overlap},      where \bx ∈ \R^{2dm}
// 
// Inputs:
//      V   #V x 3,
//      Tet #Tet x 4
//      F   #F x 3,
//      C   (m+1) x 3          vertex position for joint (assume 1 tree)
//      BE  m x 2              hande edge indexed into C
//              linear blend skinning weight matrix
//      W   #V x m(d+1)
//      config                 customizable parameters that controls behavior of the optimization
//
//  Outputs:
//      T   (d+1)m x d
//          handle transformations
//          Note d is 3 for both 2D/3D case
//      U   #V x 3.
//          deformed vertex location, under LBS with `T`
//      where   d is dimension
//              m is number of handles
//
//  Returns:
//      minimum energy function ...
float reduce_support(
    const Eigen::MatrixXf& V,
    const Eigen::MatrixXi& Tet,
    const Eigen::MatrixXi& F,
    const Eigen::MatrixXf& C,
    const Eigen::MatrixXi& BE,
    const Eigen::MatrixXf& W,
    ReduceSupportConfig<float>& config,
    Eigen::MatrixXf& T,
    Eigen::MatrixXf& U);

double reduce_support(
    const Eigen::MatrixXd& V,
    const Eigen::MatrixXi& Tet,
    const Eigen::MatrixXi& F,
    const Eigen::MatrixXd& C,
    const Eigen::MatrixXi& BE,
    const Eigen::MatrixXd& W,
    ReduceSupportConfig<double>& config,
    Eigen::MatrixXd& T,
    Eigen::MatrixXd& U);


#endif