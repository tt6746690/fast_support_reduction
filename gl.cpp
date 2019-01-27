#include <Eigen/Core>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <igl/readOBJ.h>
#include <igl/ortho.h>
#include <igl/centroid.h>
#include <igl/opengl/vertex_array.h>
#include <igl/opengl/report_gl_error.h>

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include "src/time_utils.h"
#include "src/print_opengl_info.h"
#include "src/shader.h"
#include "src/box.h"

using namespace Eigen;
using namespace std;

int scr_width  = 800;
int scr_height = 800;

string filename;
string data_dir   = "../data/";
string shader_dir = "../src/shaders/";

const auto getfilepath = [](const string& name, const string& ext){ 
    return data_dir + name + "." + ext; 
};

MatrixXd V, Vbox;
MatrixXi F, Fbox;

GLuint vao, vao_box;

bool wire_frame = false;
bool mouse_down = false;

Affine3f model = Affine3f::Identity();
Affine3f view = Affine3f::Identity() * Translation3f(Vector3f(0,0,-1));
Matrix4f projection = Matrix4f::Identity();

const auto make_centered_model = [](Affine3f& model){
    double vol;
    RowVector3d centroid;
    igl::centroid(V, F, centroid, vol);
    model = Affine3f::Identity();
    model.translate(-centroid.cast<float>().transpose());
};

int main(int argc, char* argv[])
{
    filename = "small";
    if (argc > 1) { filename = string(argv[1]); }
    igl::readOBJ(getfilepath(filename, "obj"), V, F);
    box(1, Vbox, Fbox);

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
    )";
    glfwSetWindowSizeCallback(window, [](GLFWwindow* window, int width, int height){
        ::scr_width = width; ::scr_height = height;
        float near, far, ratio, top;
        near = 0.01; far = 10; top = 10;
        ratio = float(width) / float(height);
        igl::ortho(-top*ratio, top*ratio, -top, top, near, far, projection);
        std::cout<<"projection: "<<projection.matrix()<<'\n';
    });
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
                make_centered_model(model);
                view = Affine3f::Identity() * Translation3f(Vector3f(0,0,-1));
                projection = Matrix4f::Identity();
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
        std::cout<<"view: "<<view.matrix()<<'\n';
    });
    glfwSetScrollCallback(window, [](GLFWwindow* window, double xoffset, double yoffset) {
        view.matrix()(2,3) = min(max(view.matrix()(2,3)+(float)yoffset,-100.0f), 0.f);
    });

    
    auto vspaths = vector<string>({shader_dir+"simple.vs"});
    auto fspaths = vector<string>({shader_dir+"simple.fs"});
    auto shader = Shader(vspaths, fspaths);
    shader.compile();

    {   GLuint vbo, ebo;
        igl::opengl::vertex_array(V, F, vao, vbo, ebo);
        igl::opengl::report_gl_error("vertex_array");
    }
    make_centered_model(model);

    {   GLuint vbo, ebo;
        igl::opengl::vertex_array(Vbox, Fbox, vao_box, vbo, ebo);
        igl::opengl::report_gl_error("vertex_array");
    }

    // glEnable(GL_DEPTH_TEST);

    while (!glfwWindowShouldClose(window)) 
    {
        double tic = get_seconds();
        glClearColor(0.5,0.5,0.5,1.);
        glClear(GL_COLOR_BUFFER_BIT);

        shader.compile();
        shader.use();
        shader.set_mat4("proj", projection);
        shader.set_mat4("view", view.matrix());
        shader.set_mat4("model", model.matrix());

        if (wire_frame)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        else
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

        glBindVertexArray(vao);
        glDrawElements(GL_TRIANGLES, F.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);

        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        glBindVertexArray(vao_box);
        glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_INT, 0);
        glDrawElements(GL_LINE_LOOP, 4, GL_UNSIGNED_INT, (void*)(4*sizeof(GLuint)));
        glDrawElements(GL_LINES, 8, GL_UNSIGNED_INT, (void*)(8*sizeof(GLuint)));
        glBindVertexArray(0);
 
        glfwSwapBuffers(window);
        glfwPollEvents();
        sleep_by_fps(60, tic);
    }

    glDeleteVertexArrays(1, &vao);
    glDeleteVertexArrays(1, &vao_box);
    glfwDestroyWindow(window);
    glfwTerminate();
    return EXIT_SUCCESS;
}