#include <Eigen/Core>

#include <igl/readMESH.h>
#include <igl/readOBJ.h>
#include <igl/readDMAT.h>
#include <igl/readTGF.h>
#include <igl/writeMESH.h>
#include <igl/writeOBJ.h>
#include <igl/slice.h>
#include <igl/jet.h>
#include <igl/opengl/glfw/Viewer.h>
#include <igl/normalize_row_sums.h>
#include <igl/deform_skeleton.h>
#include <igl/copyleft/cgal/convex_hull.h>
#include <igl/centroid.h>
#include <igl/project.h>
#include <igl/unproject.h>
#include <igl/snap_points.h>
#include <igl/unproject_onto_mesh.h>
#include <igl/directed_edge_parents.h>
#include <igl/forward_kinematics.h>
#include <igl/lbs_matrix.h>
// #include <igl/deform_skeleton.h>

#include "reduce_support.h"
#include "overhang_energy.h"
#include "minitrace.h"
#include "support_polygon.h"

#include <cstdio>
#include <iostream>
#include <string>
#include <algorithm>
#include <numeric>
#include <vector>
#include <cmath>
#include <unistd.h>
#include <stack>

using namespace std;
using namespace Eigen;



class State
{
public:
    using RotationList = std::vector<
        Eigen::Quaterniond, Eigen::aligned_allocator<Eigen::Quaterniond>>;
public:
    State(const Eigen::MatrixXd& C, const Eigen::MatrixXi& BE)
    {
        reset(C, BE);
    }

    // given joint relative rotations, 
    //      construct per-bone transformation matrices `T` 
    inline void compute_T() {
        RotationList dQ;
        for (int i = 0; i < BE.rows(); ++i) {
            dQ.emplace_back(
                Eigen::AngleAxisf(E(i, 0), Eigen::Vector3f::UnitX()) *
                Eigen::AngleAxisf(E(i, 1), Eigen::Vector3f::UnitY()) *
                Eigen::AngleAxisf(E(i, 2), Eigen::Vector3f::UnitZ())
            );
        }
        igl::forward_kinematics(C, BE, P, dQ, T);
    }

    // Deform skeleton, where local translation is not allowed
    inline void deform_skeleton(
        Eigen::MatrixXd& CT,
        Eigen::MatrixXi& BET) 
    {
        CT.resize(C.rows(),C.cols());
        BET.resize(BE.rows(),2);
        BET = BE;
        for(int e = 0;e<BE.rows();e++)
        {
            Matrix4d t;
            t << T.block(e*4,0,4,3).transpose(), 0,0,0,0;
            Affine3d a;
            a.matrix() = t;
            Vector3d c0 = C.row(BE(e,0));
            Vector3d c1 = C.row(BE(e,1));
            CT.row(BE(e,0)) = a * c0;
            CT.row(BE(e,1)) = a * c1;
        }
    }

    inline void reset(
        const Eigen::MatrixXd& C_,
        const Eigen::MatrixXi& BE_) 
    {
        C = C_; BE = BE_;
        T.resize(BE.rows()*4,3);
        for (int i = 0; i < BE.rows(); ++i) {
            T.block(i*3, 0, 4, 3) << 1,0,0,
                                     0,1,0,
                                     0,0,1,
                                     0,0,0;
        }
        E = Eigen::MatrixXd::Zero(BE.rows(), 3);
        igl::directed_edge_parents(BE, P);
    }

public:
    // #C by 3      list of joint positions
    //              Fixed for a given mesh.
    Eigen::MatrixXd C;
    // #BE by 2     list of bone edge indices
    Eigen::MatrixXi BE;
    // #BE          list of parent indices into BE
    Eigen::VectorXi P;

    // #BE by 3     list of euler angle for each bone
    Eigen::MatrixXd E;
    // #BE*(dim+1) by dim   stack of transposed transformation matrices
    Eigen::MatrixXd T;
};



const auto deg2rad = [](double degree) {
    return (degree / 180) * M_PI; 
};

const auto rad2deg = [](double radian) {
    return radian * 180 / M_PI;
};

string filename, data_dir;
bool is3d, show_help;
int n_fixed_bones, pso_iters, pso_population;
double c_arap, c_overhang, c_intersect, rotation_angle;
Eigen::RowVector3d dp;  // printing direction
Eigen::RowVector3d last_mouse;

MatrixXd V, U, W, C, T;
MatrixXi Tet, F, BE;

int selected = 0, joint_sel = -1;
const RowVector3d red(1., 0., 0.);
const RowVector3d green(0., 1., 0.);
const RowVector3d blue(0., 1., 0.);
const RowVector3d sea_green(70./255.,252./255.,167./255.);
const RowVector3d purple(0.5,0,0.5);

int main(int argc, char*argv[]) 
{
    show_help      = false;
    data_dir       = "../data/";
    filename       = "bb-bunny";
    n_fixed_bones  = 0;
    pso_iters      = 1;
    pso_population = 1;
    dp             = Eigen::RowVector3d(0., 1., 0.);
    c_arap         = 1;
    c_overhang     = 1;
    c_intersect    = 1;
    rotation_angle = 30;    // degree

    string usage = R"(
        usage ./support_reduction
            -d <data_dir>           Data Directory containing mesh/bone/weights etc.
            -f <filename>           Filename under data/. `filename{.mesh, .obj, .dmat, .tgf}`
            -b <n_fixed_bones>      Number of bones to be fixed, from bottom up 
            -i <pso_iters>          Number of particle swarm optimization iterations
            -p <pso_population>     Size of particle swarm optimization population
            -r <rotation_angle>     Maximum rotation (in degrees) of bones allowed
            -a <c_arap>             Coefficient for as-rigid-as-possible energy
            -c <c_overhang>         Coefficient for overhanging energy
            -e <c_intersect>        Coefficient for self-intersection energy
    )";

    int c;  
    while ((c = getopt (argc, argv, "d:f:b:i:p:r:a:c:e:")) != -1) {
        switch (c) {
            case 'd':
                data_dir = std::string(optarg);
                break;
            case 'f':
                filename = std::string(optarg);
                break;
            case 'b':
                n_fixed_bones = std::stoi(std::string(optarg));
                break;
            case 'i':
                pso_iters = std::stoi(std::string(optarg));
                break;
            case 'p':
                pso_population = std::stoi(std::string(optarg));
                break;
            case 'r':
                rotation_angle = std::stod(std::string(optarg));
                break;
            case 'a':
                c_arap = std::stod(std::string(optarg));
                break;
            case 'c':
                c_overhang = std::stod(std::string(optarg));
                break;
            case 'e':
                c_intersect = std::stod(std::string(optarg));
                break;
            case '?':
            default:
                std::cout<<usage;
                return 1;
        }
    }

    mtr_init("trace.json");

    const auto getfilepath = [&](const string& name, const string& ext){ 
        return data_dir + name + "." + ext; };

    is3d = (filename.find("woody") == 0 || filename.find("thin") == 0) 
        ? false : true;


    if (is3d) igl::readMESH(getfilepath(filename, "mesh"), V, Tet, F);
    else      igl::readOBJ(getfilepath(filename, "obj"), V, F);

    igl::readDMAT(getfilepath(filename, "dmat"), W);
    igl::normalize_row_sums(W, W);  // normalization before LBS !!
    igl::readTGF(getfilepath(filename, "tgf"), C, BE);


    {   // map vertex position to first quadrant
        RowVector3d min = V.colwise().minCoeff();
        V = V.rowwise() - min;
        C = C.rowwise() - min;
    }

    U = V;

    // find row indices to BE to be fixed
    std::vector<int> fixed_bones;
    if (BE.rows() != 0 && n_fixed_bones != 0) {
        assert(n_fixed_bones <= BE.rows());
        std::vector<double> proj_dist(BE.rows());
        for (int i = 0; i < BE.rows(); ++i) {
            proj_dist[i] = ((C.row(BE(i, 0)) + C.row(BE(i, 1))) / 2).dot(dp);
        }

        fixed_bones.resize(BE.rows());
        std::iota(fixed_bones.begin(), fixed_bones.end(), 0);
        std::partial_sort(fixed_bones.begin(), fixed_bones.begin()+n_fixed_bones, fixed_bones.end(),
            [&proj_dist](int i, int j) { return proj_dist[i] < proj_dist[j]; });
        fixed_bones.resize(n_fixed_bones);
    }

    Eigen::MatrixXd P;
    Eigen::MatrixXi G;
    support_polygon(V, 1, P, G);
    P.rowwise() -= RowVector3d(0, (V.maxCoeff() - V.minCoeff())*0.05, 0);

    Eigen::MatrixXd M;
    igl::lbs_matrix(V, W, M);


    // Undo Management
    State s(C, BE);
    std::stack<State> undo_stack,redo_stack;
    const auto push_undo = [&]() {
        undo_stack.push(s);
        redo_stack = std::stack<State>();
    };
    const auto undo = [&]() {
        if(!undo_stack.empty()) {
            redo_stack.push(s);
            s = undo_stack.top();
            undo_stack.pop();
        }
    };
    const auto redo = [&]() {
        if(!redo_stack.empty()) {
            undo_stack.push(s);
            s = redo_stack.top();
            redo_stack.pop();
        }
    };


    ReduceSupportConfig<double> config;
    config.is3d = is3d;
    config.alpha_max = 0.25 * M_PI;
    config.dp = dp;
    config.rotation_angle = deg2rad(rotation_angle);
    config.fixed_bones = fixed_bones;
    config.pso_iters = pso_iters;
    config.pso_population = pso_population;
    config.c_arap = c_arap;
    config.c_overhang = c_overhang;
    config.c_intersect = c_intersect;
    config.display = true;

    std::cout<<"rotation_angle = "<<config.rotation_angle<<'\n';


    //////////////////////////////////////////////////////////////////////
    //      Viewer
    //////////////////////////////////////////////////////////////////////

    igl::opengl::glfw::Viewer viewer;

    //  obj_id       handle to mesh for `.obj`
    //  ground_id   handle to convex hull of mesh
    int obj_id, ground_id;
    {
        // clear all ViewerData
        viewer.selected_data_index = viewer.data_list.size()-1;
        while(viewer.erase_mesh(viewer.selected_data_index)){};
        viewer.data().clear();
        
        obj_id = viewer.append_mesh();
        viewer.data().set_mesh(U, F);
        ground_id = viewer.append_mesh();
        viewer.data().set_mesh(P, G);
        viewer.selected_data_index = viewer.mesh_index(obj_id);
    }
    const auto draw_ground = [&](igl::opengl::glfw::Viewer& viewer) {
        Vector3d center;
        igl::centroid(U, F, center);
        center(1) = P.col(1).minCoeff();
        
        viewer.selected_data_index = viewer.mesh_index(ground_id);
        viewer.data().clear();
        viewer.data().set_mesh(P, G);
        viewer.data().add_points(center.transpose(), ::red);
        viewer.selected_data_index = viewer.mesh_index(obj_id);
    };

    const auto set_color = [&](igl::opengl::glfw::Viewer &viewer) {
        Eigen::MatrixXd C;
        igl::jet(W.col(selected).eval(),true,C);
        viewer.data().set_colors(C);
    };
    const auto draw_coordsys = [&](igl::opengl::glfw::Viewer &viewer, const Eigen::MatrixXd& V) {
        Eigen::Vector3d min = V.colwise().minCoeff();
        Eigen::Vector3d max = V.colwise().maxCoeff();
        Eigen::MatrixXd VCoord(7, 3);
        VCoord << 0, 0, 0,
                min(0), 0, 0,
                max(0), 0, 0,
                0, min(1), 0,
                0, max(1), 0,
                0, 0, min(2),
                0, 0, max(2);
        viewer.data().add_points(VCoord, ::green);
        for (int i = 1; i < VCoord.rows(); ++i) {
            viewer.data().add_edges(VCoord.row(0), VCoord.row(i), ::green);
        }
    };
    const auto draw_fixed_bones = [](
        igl::opengl::glfw::Viewer &viewer, const Eigen::MatrixXd& C, const Eigen::MatrixXi& BE, std::vector<int>& fixed_bones) {
        for (int i = 0; i < fixed_bones.size(); ++i) {
            auto edge = BE.row(fixed_bones[i]);
            viewer.data().add_edges(C.row(edge(0)), C.row(edge(1)), ::red);
        }
    };
    const auto draw_bones = [](
            igl::opengl::glfw::Viewer &viewer, const Eigen::MatrixXd& C, const Eigen::MatrixXi& BE) {
        viewer.data().add_points(C, ::sea_green);
        for (int i = 0; i < BE.rows(); ++i) {
            viewer.data().add_edges(C.row(BE(i, 0)), C.row(BE(i, 1)), ::sea_green);
        }
    };
    const auto update = [&](const Eigen::MatrixXd& T) {
        s.T = T;
        U = M * s.T;
        viewer.data().clear();
        viewer.data().set_mesh(U, F);

        Eigen::MatrixXd CT;
        Eigen::MatrixXi BET;
        s.deform_skeleton(CT, BET);

        if (joint_sel != -1) {
            Eigen::MatrixXd Csel(1, 3);
            Csel.row(0) = CT.row(joint_sel);
            viewer.data().set_points(Csel, ::purple);
            draw_coordsys(viewer, U);
            draw_bones(viewer, CT, BET);
            draw_fixed_bones(viewer, CT, BET, fixed_bones);
            viewer.data().add_points(Csel, ::purple);
        } else {
            draw_coordsys(viewer, U);
            draw_bones(viewer, CT, BET);
            draw_fixed_bones(viewer, CT, BET, fixed_bones);
        }
        draw_ground(viewer);
    };

    viewer.data().show_lines = false;
    viewer.data().show_overlay_depth = false;
    viewer.data().line_width = 1000;
    viewer.data().point_size = 15;


    viewer.callback_mouse_down = [&](igl::opengl::glfw::Viewer&, int, int) {
        last_mouse = Eigen::RowVector3d(
            viewer.current_mouse_x,viewer.core.viewport(3)-viewer.current_mouse_y,0);  
        
        Eigen::MatrixXd CT;
        Eigen::MatrixXi BET;
        s.deform_skeleton(CT, BET);

        // joint projected to screen space 
        Eigen::MatrixXd CP;
        igl::project(CT, viewer.core.view, viewer.core.proj, viewer.core.viewport, CP);
        Eigen::VectorXd D = (CP.rowwise()-last_mouse).rowwise().norm();
        joint_sel = (D.minCoeff(&joint_sel) < 30)?joint_sel:-1;
        if (joint_sel != -1) {
            std::cout<<"joint selected: "<<joint_sel<<'\n';
            last_mouse(2) = CT(joint_sel, 2);
            push_undo();
            s.compute_T();
            update(s.T);
            return true;
        }
        return false;
    };
    viewer.callback_key_down = [&](igl::opengl::glfw::Viewer&, unsigned char key, int mod) {
        std::cout<<"key pressed: " << key <<'\n';
        switch(key) {
            case '>': {
                set_color(viewer);
                selected++;
                selected = std::min(std::max(selected,0),(int)W.cols()-1);
                break;
            }
            case '<': {
                set_color(viewer);
                selected--;
                selected = std::min(std::max(selected,0),(int)W.cols()-1);
                break;
            }
            case 'R':
            case 'r': {
                push_undo();
                s.reset(C, BE);
                s.compute_T();
                update(s.T);
                break;
            }
            case 'D':
            case 'd': {
                joint_sel = -1;
                s.compute_T();
                update(s.T);
                break;
            }
            case 'G':
            case 'g': {
                std::cout<<"Starting optimization\n";
                Eigen::MatrixXd C_, V_;
                Eigen::MatrixXi BE_;
                s.compute_T();
                V_ = M * s.T;
                s.deform_skeleton(C_,BE_);

                Eigen::MatrixXd E;
                reduce_support(V_, Tet, F, C_, BE_, W, config, T, U, E);

                for (int i = 0; i < E.rows(); ++i) {
                    s.E.row(i) += E.row(i);
                }
                s.compute_T();

                update(s.T);
                push_undo();
                s.reset(C, BE);
                break;
            }
            case 'S':
            case 's': {
                auto outfile = getfilepath(filename+"_deformed_"+std::to_string(pso_iters), "obj");
                igl::writeOBJ(outfile, U, F);
                std::cout<<"Saving model to " << outfile << '\n';
                break;
            }
            case 'H':
            case 'h': {
                if (joint_sel != -1) {
                    s.E(joint_sel, 0) += (mod != GLFW_MOD_SHIFT) ?
                         1./18*M_PI :
                        -1./18*M_PI;
                    std::cout<<"euler angle ("<<joint_sel<<", "<<0<<") = "<<s.E(joint_sel, 0)<<'\n';
                    s.compute_T();
                    update(s.T);
                }
                break;
            }
            case 'J':
            case 'j': {
                if (joint_sel != -1) {
                    s.E(joint_sel, 1) += (mod != GLFW_MOD_SHIFT) ?
                         1./18*M_PI :
                        -1./18*M_PI;
                    std::cout<<"euler angle ("<<joint_sel<<", "<<1<<") = "<<s.E(joint_sel, 1)<<'\n';
                    s.compute_T();
                    update(s.T);
                }
                break;
            }
            case 'K':
            case 'k': {
                if (joint_sel != -1) {
                    s.E(joint_sel, 2) += (mod != GLFW_MOD_SHIFT) ?
                         1./18*M_PI :
                        -1./18*M_PI;
                    std::cout<<"euler angle ("<<joint_sel<<", "<<2<<") = "<<s.E(joint_sel, 2)<<'\n';
                    s.compute_T();
                    update(s.T);
                }
                break;
            }
        }
        return true;
    };

    s.compute_T();
    update(s.T);
    draw_ground(viewer);
    draw_bones(viewer, C, BE);
    draw_coordsys(viewer, U);
    draw_fixed_bones(viewer, C, BE, fixed_bones);

    std::cout<<R"(
[click]     Select joint
D,d         Deselect joint
R,r         Reset joints
⌘ Z         Undo
⌘ ⇧ Z       Redo
>           Show next weight function.
<           Show previous weight function.
G,g         Start Optimization.
S,s         Save `.obj` file
H,h/<sh>    Increase/Decrease Euler's angle about x-axis
J,j/<sh>    Increase/Decrease Euler's angle about y-axis
K,k/<sh>    Increase/Decrease Euler's angle about z-axis
)";
    viewer.launch();
    mtr_flush();
    mtr_shutdown();
}