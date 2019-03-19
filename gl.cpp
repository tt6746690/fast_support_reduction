#include <Eigen/Core>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <igl/png/render_to_png.h>
#include <igl/ortho.h>
#include <igl/centroid.h>
#include <igl/opengl/report_gl_error.h>
#include <igl/opengl/init_render_to_texture.h>
#include <igl/look_at.h>
#include <igl/readOBJ.h>
#include <igl/readMESH.h>
#include <igl/readDMAT.h>
#include <igl/normalize_row_sums.h>
#include <igl/lbs_matrix.h>

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include <igl/opengl/glfw/background_window.h>

#include "time_utils.h"
#include "print_opengl_info.h"
#include "Shader.h"
#include "persp.h"
#include "wrap_igl.h"
#include "normalized_device_coordinate.h"
#include "depthbuffer_to_png.h"
#include "intersection_volume.h"
#include "Line.h"
#include "Quad.h"
#include "Box.h"

#include "minitrace.h"

#ifdef __APPLE__
    #define GL_SILENCE_DEPRECATION
#endif


using namespace Eigen;
using namespace std;

// sizse for on-screen rendering
int scr_width  = 800;
int scr_height = 800;

// size for off-screen rendering to texture
//      note: scr_{width, height} changes with window resize callback
const int ren_width_  = 400;
const int ren_height_ = 400;

string filename;
string data_dir   = "../data/";
string shader_dir_ = "../src/shaders/";

const auto getfilepath = [](const string& name, const string& ext){ 
    return data_dir + name + "." + ext; 
};

MatrixXf V, W, M, T;
MatrixXi F;
MatrixXi Tet; // !!!!!!!!!!!

GLuint vao;

bool wire_frame = false;
bool orthographic = false;
bool mouse_down = false;
bool save_png = true;
bool compute_selfintersection = true;

Affine3f model = Affine3f::Identity();
Affine3f view = Affine3f::Identity() * Translation3f(Vector3f(0, 0, -10));
Matrix4f projection = Matrix4f::Identity();

// a bit > 1 so that gl_FragCoord.z == 1 means reached far plane
//      instead of the added possibility of a fragment just touching the far plane
float half = 1.001;

const auto set_view = [](Affine3f& view) {
    if (orthographic) {
        igl::look_at(Vector3f(0,0,-half), Vector3f(0,0,0), Vector3f(0,1,0), view.matrix());
    } else {
        view = Affine3f::Identity() * Translation3f(Vector3f(0, 0, -10));
    }
};

// need to call this before setting glViewPort
//      MacOS retina screen framebuffer size is twice that of window size
const auto set_viewport = [](GLFWwindow* window) {
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    glViewport(0, 0, width, height);
};

const auto make_centered_model = [](Affine3f& model){
    double vol;
    RowVector3f centroid;
    igl::centroid(V, F, centroid, vol);
    model = Affine3f::Identity();
    model.translate(-centroid.transpose());
};

const auto reshape = [](GLFWwindow* window, int width, int height) {
    ::scr_width = width; ::scr_height = height;
    float near, far, top, bottom, left, right;
    if (orthographic) {
        float near, far, top, right, left, bottom;
        near = -half; far = half; top = half;
        right = top * (double)::scr_width/(double)::scr_height;
        left = -right; bottom = -top;
        igl::ortho(left, right, bottom, top, near, far, projection);
    } else {
        near = 0.01;
        far = 100;
        top = tan(35./360.*M_PI)*near;
        right = top * (double)::scr_width/(double)::scr_height;
        left = -right;
        bottom = -top;
        persp(left, right, bottom, top, near, far, projection);
    }
};

const auto reshape_current = [](GLFWwindow* window) {
    int width_window, height_window;
    glfwGetWindowSize(window, &width_window, &height_window);
    reshape(window,width_window,height_window);
};



int main(int argc, char* argv[])
{

    filename = "small";
    if (argc > 1) { filename = string(argv[1]); }
    igl::readMESH(getfilepath(filename, "mesh"), V, Tet, F);
    igl::readDMAT(getfilepath(filename, "dmat"), W);
    igl::normalize_row_sums(W, W);  // normalization before LBS !!
    normalized_device_coordinate(V);


    Eigen::MatrixXd Vd = V.cast<double>().eval();
    Eigen::MatrixXd Wd = W.cast<double>().eval();
    Eigen::MatrixXd Md;
    igl::lbs_matrix(Vd, Wd, Md);
    Eigen::MatrixXf M;
    M = Md.cast<float>().eval();


    // initialize objects
    SelfIntersectionVolume vol(V, W, M, F, ren_width_, ren_height_, shader_dir_);

    auto screen = Quad<float>();
    auto xaxis = Line<float>(Vector3f(-1,0,0), Vector3f(5,0,0));
    auto yaxis = Line<float>(Vector3f(0,-1,0), Vector3f(0,5,0));
    auto unitbox = Box<float>(half);

    if (!glfwInit()) { std::cerr<<"Could not initialize glfw\n"; return -1; }
    glfwSetErrorCallback([](int err, const char* msg) { std::cerr<<msg<<'\n'; });
    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_VISIBLE, GL_TRUE);

    GLFWwindow* window = glfwCreateWindow(scr_width, scr_height, "gl", NULL, NULL);
    if (window == NULL) { 
        std::cerr<<"Failed to create GLFW window"<<'\n';
        glfwTerminate(); 
        return EXIT_FAILURE; }
    glfwMakeContextCurrent(window);
    glfwSetInputMode(window,GLFW_CURSOR,GLFW_CURSOR_NORMAL);

    if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
        std::cerr<<"Failed to load OpenGL and its extensions\n";
        return EXIT_FAILURE;
    }

    print_opengl_info(window);
    igl::opengl::report_gl_error("init");


    std::cout<< R"(
    Usage:
    L,l     toggle wireframe rendering
    Z,z     reset view to look along z-axis
    R,r     reset model, view, projection to default
    P,p     toggle orthographic viewings
    S,s     save color/depth to .png
    )";
    glfwSetWindowSizeCallback(window, reshape);
    reshape_current(window);

    glfwSetFramebufferSizeCallback(window, [](GLFWwindow* window, int width, int height) {
        glViewport(0, 0, width, height);
    });

    glfwSetKeyCallback(window, [](GLFWwindow* window, int key, int scancode, int action, int mods) {
        if(key == GLFW_KEY_ESCAPE && action == GLFW_PRESS)
            glfwSetWindowShouldClose(window,true);
    });

    glfwSetCharModsCallback(window, [](GLFWwindow* window, unsigned int codepoint, int mods) {
        std::cout<<"key: "<<static_cast<char>(codepoint)<<'\n';
        switch(codepoint) {
            case 'L':
            case 'l': 
                wire_frame ^= 1; break;
            case 'Z':
            case 'z': 
                view.matrix().block(0,0,3,3).setIdentity(); break;
            case 'R':
            case 'r':
                set_view(view);
                reshape_current(window);
                break;
            case 'P':
            case 'p':
                orthographic ^= 1; 
                set_view(view);
                reshape_current(window);
                break;
            case 'S':
            case 's':
                save_png ^= 1;
                break;
            case 'C':
            case 'c':
                compute_selfintersection ^= 1;
                break;
            default:
                std::cout<<"Unrecognized key: "<<static_cast<char>(codepoint)<<'\n';
                break;
        }
    });
    glfwSetMouseButtonCallback(window, [](GLFWwindow* window, int button, int action, int mods){
        mouse_down = action == GLFW_PRESS;
    });
    glfwSetCursorPosCallback(window, [](GLFWwindow* window, double x, double y) {
        static double mouse_last_x = x;
        static double mouse_last_y = y;
        double dx = x-mouse_last_x;
        double dy = y-mouse_last_y;
        if(mouse_down) {
            float factor = abs(view.matrix()(2,3));
            view.rotate(AngleAxisf(dx*factor/float(::scr_width), Vector3f(0,1,0)));
            view.rotate(AngleAxisf(dy*factor/float(::scr_height),
                view.matrix().topLeftCorner(3,3).inverse() * Vector3f(1,0,0)));
        }
        mouse_last_x = x;
        mouse_last_y = y;
    });
    glfwSetScrollCallback(window, [](GLFWwindow* window, double xoffset, double yoffset) {
        view.matrix()(2,3) = min(max(view.matrix()(2,3)+(float)yoffset,-100.0f), 0.f);
    });

    auto viz_shader = Shader({shader_dir_+"viz.vs"}, {shader_dir_+"viz.fs"});
    auto screen_shader = Shader({shader_dir_+"screen.vs"}, {shader_dir_+"screen.fs"});
    viz_shader.compile();
    screen_shader.compile();

    vertex_array(V, F, vao);
    screen.create_vertex_array();
    xaxis.create_vertex_array();
    yaxis.create_vertex_array();
    unitbox.create_vertex_array();

    vol.prepare();

    // compute view and projection matices
    // Notice: could not be put inside prepare() based on current design
    Eigen::RowVector3f A_center = 0.5*(V.colwise().maxCoeff() + V.colwise().minCoeff());
    float new_half = (V.rowwise()-A_center).rowwise().norm().maxCoeff() * 1.0001;
    igl::ortho(-new_half, new_half, -new_half, new_half, 0, 2*new_half, vol.projection);
    Eigen::Vector3f eye(A_center(0), A_center(1), -new_half+A_center(2));
    Eigen::Vector3f target(A_center(0), A_center(1), A_center(2));;
    Eigen::Vector3f up(0, 1, 0);
    igl::look_at(eye, target, up, vol.view.matrix());
    vol.model = Eigen::Affine3f::Identity();
    vol.ortho_box_volume = pow(2*new_half, 3.0);

    while (!glfwWindowShouldClose(window))
    {
        double tic = get_seconds();

        if (compute_selfintersection) {
            float volume = vol.compute(vol.T);
            std::cout << "Volume: " << volume << std::endl;
            compute_selfintersection = false;
        }

        //  default framebuffer
        glBindFramebuffer(GL_FRAMEBUFFER, 0);
        set_viewport(window);
        glClearColor(0.5, 0.5, 0.5, 1.);
        glClearDepth(1.);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        if (wire_frame)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        else
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        viz_shader.compile();
        viz_shader.use();
        viz_shader.set_mat4("proj",  projection);
        viz_shader.set_mat4("view",  view.matrix());
        viz_shader.set_mat4("model", model.matrix());

        glBindVertexArray(vao);
        glDrawElements(GL_TRIANGLES, F.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);

        xaxis.draw();
        yaxis.draw();
        unitbox.draw();

        screen_shader.compile();
        screen_shader.use();
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        for (int i = 0; i < 2; ++i) {
            screen_shader.set_mat4("mvp", (projection*view*(model*Translation3f(Vector3f(0,0,-2-2*i)))).matrix());
            screen_shader.set_int("screen_texture", i);
            glActiveTexture(GL_TEXTURE0+i);
            glBindTexture(GL_TEXTURE_2D, vol.ren_tex[i]);
            screen.draw();
        }

        glfwSwapBuffers(window);
        glfwPollEvents();
        sleep_by_fps(60, tic);
    }

    glDeleteVertexArrays(1, &vao);
    glfwDestroyWindow(window);
    glfwTerminate();

    return EXIT_SUCCESS;
}