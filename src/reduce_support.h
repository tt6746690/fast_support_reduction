#ifndef __REDUCE_SUPPORT_H__
#define __REDUCE_SUPPORT_H__
#include <Eigen/Core>
#include <Eigen/Sparse>


// Configuration that controls behavior of optimization procedure
template <typename T>
class ReduceSupportConfig{
public:

    // Default ctor
    ReduceSupportConfig()
        :   is3d(true),
            alpha_max(0.25*M_PI), 
            dp(0.,1.,0.),
            fixed_bones(),
            rotation_angle(0.25*M_PI),
            pso_iters(1),
            pso_population(1),
            c_arap(1),
            c_overhang(1),
            c_intersect(1),
            unsafe(),
            display(false)
        {}

    // Conversions

    template <typename U>
    ReduceSupportConfig(const ReduceSupportConfig<U>& that) {
        cast(that, *this);
    }
    template <typename U>
    ReduceSupportConfig<T>& operator=(const ReduceSupportConfig<U>& that) {
        cast(that, *this);
        return *this;
    }

private:

    template <typename U>
    void cast(const ReduceSupportConfig<U>& from, ReduceSupportConfig<T>& to) {
        to.is3d           =     from.is3d;
        to.alpha_max      = (T) from.alpha_max;
        to.dp             =     from.dp.template cast<T>();
        to.fixed_bones    =     from.fixed_bones;
        to.rotation_angle = (T) from.rotation_angle;
        to.pso_iters      =     from.pso_iters;
        to.pso_population =     from.pso_population;
        to.c_arap         = (T) from.c_arap;
        to.c_overhang     = (T) from.c_overhang;
        to.c_intersect    = (T) from.c_intersect;
        to.display        =     from.display;
        to.unsafe         =     from.unsafe;   
    }

public:

    // dimension of input mesh 
    bool is3d;

    //  maximal self-supporting angle
    T alpha_max;
    //  normalized printing direction
    Eigen::Matrix<T, 3, 1> dp;

    // bones that are fixed, for "sticking" the feet to the printing platform
    //      i.e. rows of `BE` from .tgf format
    std::vector<int> fixed_bones;
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

    // #risky x 2       risky edges recorded during overhang energy
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