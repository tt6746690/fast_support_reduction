#include <Eigen/Core>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <igl/png/render_to_png.h>
#include <igl/readOBJ.h>
#include <igl/ortho.h>
#include <igl/centroid.h>
#include <igl/opengl/report_gl_error.h>
#include <igl/opengl/init_render_to_texture.h>

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include "time_utils.h"
#include "print_opengl_info.h"
#include "Shader.h"
#include "persp.h"
#include "wrap_igl.h"
#include "primitives.h"
#include "vertex_array_with_texture.h"
#include "init_render_to_texture.h"
#include "depthbuffer_to_png.h"
#include "camera.h"
#include "Line.h"
#include "Box.h"

using namespace Eigen;
using namespace std;

// sizse for on-screen rendering
int scr_width  = 800;
int scr_height = 800;

// size for off-screen rendering to texture
int ren_width = 800;
int ren_height = 800;

string filename;
string data_dir   = "../data/";
string shader_dir = "../src/shaders/";

const auto getfilepath = [](const string& name, const string& ext){ 
    return data_dir + name + "." + ext; 
};

MatrixXf V, Vscreen, Tscreen;
MatrixXi F, Fscreen;

GLuint vao, vao_screen;

bool wire_frame = false;
bool orthographic = false;
bool mouse_down = false;
bool save_png = true;

Affine3f model = Affine3f::Identity();
Affine3f view = Affine3f::Identity() * Translation3f(Vector3f(0, 0, -10));
Matrix4f projection = Matrix4f::Identity();

float half = 1.5;

const auto set_view = [](Affine3f& view) {
    if (orthographic) {
        // view = lookat_view(Vector3f(0,0,half), Vector3f(0,0,0), Vector3f(0,1,0));
        view = Affine3f::Identity();
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


// map vertex position to normalized coordinates [-1,1]^3
const auto normalize_coordinate = [](MatrixXf& V) {
    RowVectorXf min, max, diff;
    max = V.colwise().minCoeff();
    min = V.colwise().maxCoeff();
    diff = max - min;
    for (int i = 0; i < V.rows(); ++i) {
        for (int j = 0; j < V.cols(); ++j) {
            V(i, j) = 2 * (V(i, j) - min(j)) / diff(j) - 1;
        }
    }
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
    igl::readOBJ(getfilepath(filename, "obj"), V, F);
    normalize_coordinate(V);

    textured_quad(Vscreen, Fscreen, Tscreen);
    Line<float> xaxis(Vector3f(-1,0,0), Vector3f(5,0,0));
    Line<float> yaxis(Vector3f(0,-1,0), Vector3f(0,5,0));
    Box<float> unitbox(1.);


    Matrix4f ortho_proj;
    float near, far, top, right, left, bottom;
    near = -half; far = half; top = half;
    right = top * (double)::scr_width/(double)::scr_height;
    left = -right; bottom = -top;
    igl::ortho(left, right, bottom, top, near, far, ortho_proj);
    // Affine3f ortho_view = lookat_view(Vector3f(0,0,0.5*near), Vector3f(0,0,0), Vector3f(0,1,0));
    Affine3f ortho_view = Affine3f::Identity();

    if (!glfwInit()) { std::cerr<<"Could not initialize glfw\n"; return -1; }
    glfwSetErrorCallback([](int err, const char* msg) { std::cerr<<msg<<'\n'; });
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

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

    auto peel_shader = Shader({shader_dir+"peel.vs"}, {shader_dir+"peel.fs"});
    auto viz_shader = Shader({shader_dir+"viz.vs"}, {shader_dir+"viz.fs"});
    auto screen_shader = Shader({shader_dir+"screen.vs"}, {shader_dir+"screen.fs"});
    
    peel_shader.compile();
    viz_shader.compile();
    screen_shader.compile();


    vertex_array(V, F, vao);
    vertex_array_texture(Vscreen, Fscreen, Tscreen, vao_screen);
    xaxis.create_vertex_array();
    yaxis.create_vertex_array();
    unitbox.create_vertex_array();

    // create framebuffer
    GLuint fbo, tex_id, dtex_id;
    init_render_to_texture(scr_width, scr_height, fbo, tex_id, dtex_id);


    glEnable(GL_DEPTH_TEST);
    glDepthMask(GL_TRUE);
    glDepthFunc(GL_LESS);   // d_frag < d_fbo
    glDepthRange(0, 1);     // linearly map: [-1, 1] (normalized coordinates) -> [0, 1] (screen)
    glDisable(GL_CULL_FACE);

    while (!glfwWindowShouldClose(window)) 
    {
        double tic = get_seconds();

        // off-screen rendering to texture
        glBindFramebuffer(GL_FRAMEBUFFER, fbo);
        glViewport(0, 0, ren_width, ren_height);     // need to set this !
        glClearColor(0.1, 0.1, 0.1, 1.);
        glClearDepth(1.);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        peel_shader.compile();
        peel_shader.use();
        peel_shader.set_mat4("model_view_proj", (ortho_proj*ortho_view*model).matrix());
        glBindVertexArray(vao);
        glDrawElements(GL_TRIANGLES, F.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);

        if (save_png) {
            igl::png::render_to_png("color.png", ren_width, ren_height, true, false);
            depthbuffer_to_png("depth.png", ren_width, ren_height);
            save_png = false;
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
        screen_shader.set_mat4("proj",  projection);
        screen_shader.set_mat4("view",  view.matrix());
        screen_shader.set_mat4("model", (model*Translation3f(Vector3f(0,0,-2))).matrix());
        screen_shader.set_int("depth_texture", 0);

        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, dtex_id);
        glBindVertexArray(vao_screen);
        glDrawElements(GL_TRIANGLES, Fscreen.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);

 
        glfwSwapBuffers(window);
        glfwPollEvents();
        sleep_by_fps(60, tic);
    }

    glDeleteFramebuffers(1, &fbo);
    glDeleteVertexArrays(1, &vao);
    glfwDestroyWindow(window);
    glfwTerminate();
    return EXIT_SUCCESS;
}