#include "arap_energy_clustering.h"
#include <igl/project_isometrically_to_plane.h>
#include <igl/covariance_scatter_matrix.h>
#include <igl/repdiag.h>
#include <igl/cotmatrix.h>
#include <igl/speye.h>
#include <igl/slice.h>
#include <igl/mode.h>
#include <igl/group_sum_matrix.h>
#include <igl/arap_rhs.h>
#include <igl/massmatrix.h>
#include <igl/columnize.h>


template <
  typename DerivedV,
  typename DerivedF>
void arap_precompute(
    const Eigen::PlainObjectBase<DerivedV> & V,
    const Eigen::PlainObjectBase<DerivedF> & F,
    const int dim,
    igl::ARAPData & data)
{
    typedef typename DerivedV::Scalar Scalar;
    const int n = V.rows();
    data.n = n;
    // data.b = b;   no need for b anymore
    data.dim = dim;
    data.f_ext = Eigen::MatrixXd::Zero(n, data.dim);

    bool flat = (V.cols() - data.dim) == 1;

    DerivedV plane_V;
    DerivedF plane_F;
    typedef SparseMatrix<Scalar> SparseMatrixS;
    SparseMatrixS ref_map,ref_map_dim;

    if (flat) {
        igl::project_isometrically_to_plane(V, F, plane_V, plane_F, ref_map);
        igl::repdiag(ref_map, dim, ref_map_dim);
    }

    const PlainObjectBase<DerivedV>& ref_V = (flat?plane_V:V);
    const PlainObjectBase<DerivedF>& ref_F = (flat?plane_F:F);

    SparseMatrixS L;
    cotmatrix(V,F,L);

    igl::ARAPEnergyType eff_energy = data.energy;

    if (eff_energy == igl::ARAP_ENERGY_TYPE_DEFAULT)
    {
        switch(F.cols())
        {
        case 3:
            if(data.dim == 3) {
                eff_energy = igl::ARAP_ENERGY_TYPE_SPOKES_AND_RIMS;
            } else {
                eff_energy = igl::ARAP_ENERGY_TYPE_ELEMENTS;
            }
            break;
        case 4:
            eff_energy = igl::ARAP_ENERGY_TYPE_ELEMENTS;
            break;
        default:
            assert(false);
        }
    }

    // Get covariance scatter matrix, when applied collects the covariance
    // matrices used to fit rotations to during optimization
    igl::covariance_scatter_matrix(ref_V, ref_F, eff_energy, data.CSM);
    if (flat)
    {
        data.CSM = (data.CSM * ref_map_dim.transpose()).eval();
    }

    // Get group sum scatter matrix, when applied sums all entries of the same
    // group according to G
    SparseMatrix<double> G_sum;
    if (data.G.size() == 0) {
        if (eff_energy == igl::ARAP_ENERGY_TYPE_ELEMENTS) {
            igl::speye(F.rows(), G_sum);
        }
        else {
            igl::speye(n, G_sum);
        }
    }
    else {
        // groups are defined per vertex, convert to per face using mode
        if (eff_energy == igl::ARAP_ENERGY_TYPE_ELEMENTS) {
            Eigen::Matrix<int,Eigen::Dynamic,1> GG;
            Eigen::MatrixXi GF(F.rows(), F.cols());
            for (int j = 0; j<F.cols(); j++) {
                Eigen::Matrix<int,Eigen::Dynamic,1> GFj;
                igl::slice(data.G, F.col(j), GFj);
                GF.col(j) = GFj;
            }
            igl::mode<int> (GF,2,GG);
            data.G = GG;
        }
        igl::group_sum_matrix(data.G, G_sum);
    }

    Eigen::SparseMatrix<double> G_sum_dim;
    igl::repdiag(G_sum, data.dim, G_sum_dim);
    data.CSM = (G_sum_dim * data.CSM).eval();

    igl::arap_rhs(ref_V, ref_F, data.dim, eff_energy, data.K);
    if (flat) {
        data.K = (ref_map_dim * data.K).eval();
    }

    Eigen::SparseMatrix<double> Q = (-L).eval();

    if (data.with_dynamics) {
        const double h = data.h;
        Eigen::SparseMatrix<double> M;
        igl::massmatrix(V, F, igl::MASSMATRIX_TYPE_DEFAULT, data.M);
        const double dw = (1./data.ym) * (h * h);
        Eigen::SparseMatrix<double> DQ = dw * 1./ (h * h) * data.M;
        Q += DQ;
        // Dummy external forces
        data.f_ext = Eigen::MatrixXd::Zero(n, data.dim);
        data.vel = Eigen::MatrixXd::Zero(n, data.dim);
    }

}

template <
  typename Derivedbc,
  typename DerivedU>
double arap_energy (
    const Eigen::PlainObjectBase<Derivedbc>& bc,
    igl::ARAPData& data,
    Eigen::PlainObjectBase<DerivedU>& U) {

    

    const int n = data.n;
    int iter = 0;

    if (U.size() == 0) {
        // terrible initial guess.. should at least copy input mesh
        #ifndef NDEBUG
            cerr << "arap_solve: Using terrible initial guess for U. Try U = V." << endl;
        #endif
            U = Eigen::MatrixXd::Zero(data.n,data.dim);
    } else {
        assert(U.cols() == data.dim && "U.cols() match data.dim");
    }

    // changes each arap iteration
    Eigen::MatrixXd U_prev = U;
    // doesn't change for fixed with_dynamics timestep
    Eigen::MatrixXd U0;
    if (data.with_dynamics) {
        U0 = U_prev;
    }

    while (iter < data.max_iter) {
        U_prev = U;
        // enforce boundary conditions exactly
        for (int bi = 0; bi < bc.rows(); bi++) {
            U.row(data.b(bi)) = bc.row(bi);
        }

        const auto & Udim = U.replicate(data.dim, 1);
        Eigen::MatrixXd S = data.CSM * Udim;
        // THIS NORMALIZATION IS IMPORTANT TO GET SINGLE PRECISION SVD CODE TO WORK
        // CORRECTLY.
        S /= S.array().abs().maxCoeff();

        const int Rdim = data.dim;
        Eigen::MatrixXd R(Rdim, data.CSM.rows());
        if (R.rows() == 2) {
            fit_rotations_planar(S, R);
        } else {
            fit_rotations(S, true, R);
        }
    }

    // Number of rotations: #vertices or #elements
    int num_rots = data.K.cols() / Rdim / Rdim;
    // distribute group rotations to vertices in each group
    Eigen::MatrixXd eff_R;
    if (data.G.size() == 0) {
        eff_R = R;
    } else {
        eff_R.resize(Rdim, num_rots * Rdim);
        for (int r = 0;r < num_rots; r++) {
            eff_R.block(0, Rdim * r, Rdim, Rdim) = R.block(0, Rdim*data.G(r), Rdim, Rdim);
        }
    }

    

}