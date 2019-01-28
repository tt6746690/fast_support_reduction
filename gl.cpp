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
#include "src/mesh_to_vao.h"
#include "src/shader.h"
#include "src/box.h"
#include "src/icosahedron.h"


using namespace Eigen;
using namespace std;

int scr_width  = 640;
int scr_height = 360;
int highdpi = 1;
GLuint prog_id=0;

string filename;
string data_dir   = "../data/";
string shader_dir = "../src/shaders/";

const auto getfilepath = [](const string& name, const string& ext){ 
    return data_dir + name + "." + ext; 
};

Eigen::Matrix< float,Eigen::Dynamic,3,Eigen::RowMajor> V;
Eigen::Matrix< GLuint,Eigen::Dynamic,3,Eigen::RowMajor> F;

GLuint vao, VAO, vao_box;

bool wire_frame = false;
bool mouse_down = false;

Affine3f model = Affine3f::Identity();
Affine3f view = Affine3f::Identity() * Translation3f(Vector3f(0,0,-10));
Matrix4f projection = Matrix4f::Identity();

// const auto make_centered_model = [](Affine3f& model){
//     double vol;
//     RowVector3d centroid;
//     igl::centroid(V, F, centroid, vol);
//     model = Affine3f::Identity();
//     model.translate(-centroid.cast<float>().transpose());
// };

int main(int argc, char* argv[])
{
    filename = "small";
    if (argc > 1) { filename = string(argv[1]); }
    igl::readOBJ(getfilepath(filename, "obj"), V, F);
    // box(1, Vbox, Fbox);

    if (!glfwInit()) { std::cerr<<"Could not initialize glfw\n"; return -1; }

    
    glfwSetErrorCallback([](int err, const char* msg) { std::cerr<<msg<<'\n'; });
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    GLFWwindow* window = glfwCreateWindow(scr_width, scr_height, "gl", NULL, NULL);


    glfwSetWindowPos(window,0,0);
    glfwMakeContextCurrent(window);


    // Load OpenGL and its extensions
    if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress))
    {
        std::cerr<<"Failed to load OpenGL and its extensions"<<std::endl;
        return EXIT_FAILURE;
    }
    print_opengl_info(window);
    igl::opengl::report_gl_error("init");


    icosahedron(V,F);
    mesh_to_vao(V,F,VAO);
    igl::opengl::report_gl_error("mesh_to_vao");

    const auto & reshape = [](
        GLFWwindow* window,
        int _width,
        int _height)
    {
        ::scr_width=_width,::scr_height=_height;

        // augh, windows can't handle variables named near and far.
        float nearVal = 0.01;
        float farVal = 100;
        float top = tan(35./360.*M_PI)*nearVal;
        float right = top * (double)::scr_width/(double)::scr_height;
        float left = -right;
        float bottom = -top;
        // igl::ortho(left, right, bottom, top, nearVal, farVal, projection);
        projection.setConstant(4,4,0.);
        projection(0,0) = (2.0 * nearVal) / (right - left);
        projection(1,1) = (2.0 * nearVal) / (top - bottom);
        projection(0,2) = (right + left) / (right - left);
        projection(1,2) = (top + bottom) / (top - bottom);
        projection(2,2) = -(farVal + nearVal) / (farVal - nearVal);
        projection(3,2) = -1.0;
        projection(2,3) = -(2.0 * farVal * nearVal) / (farVal - nearVal);
    };
    // Set up window resizing
    glfwSetWindowSizeCallback(window,reshape);
    {
        int width_window, height_window;
        glfwGetWindowSize(window, &width_window, &height_window);
        reshape(window,width_window,height_window);
    }

    // Close the window if user presses ESC or CTRL+C
    glfwSetKeyCallback(
        window,
        [](GLFWwindow* window, int key, int scancode, int action, int mods)
        {
        if(key == 256 || (key == 67 && (mods & GLFW_MOD_CONTROL)))
        {
            glfwSetWindowShouldClose(window,true);
        }
        });

 





    // handle key events
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
                // make_centered_model(model);
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


    });


    glfwSetScrollCallback(window, [](GLFWwindow* window, double xoffset, double yoffset) {
        view.matrix()(2,3) =
        std::min(std::max(view.matrix()(2,3)+(float)yoffset,-100.0f),-2.0f);
    });

    
    auto vspaths = vector<string>({shader_dir+"simple.vs"});
    auto fspaths = vector<string>({shader_dir+"simple.fs"});
    auto shader = Shader(vspaths, fspaths);
    shader.compile();

    // {   GLuint vbo, ebo;
    //     igl::opengl::vertex_array(V, F, vao, vbo, ebo);
    //     igl::opengl::report_gl_error("vertex_array");
    // }

    icosahedron(V,F);
    mesh_to_vao(V,F,VAO);
    igl::opengl::report_gl_error("mesh_to_vao");


    // make_centered_model(model);

    // {   GLuint vbo, ebo;
    //     igl::opengl::vertex_array(Vbox, Fbox, vao_box, vbo, ebo);
    //     igl::opengl::report_gl_error("vertex_array");
    // }

    // glEnable(GL_DEPTH_TEST);

    while (!glfwWindowShouldClose(window)) 
    {
        double tic = get_seconds();
        glClearColor(0.5,0.5,0.5,0);
        glClear(GL_COLOR_BUFFER_BIT);
        glfwGetFramebufferSize(window, &::scr_width, &::scr_height);
        glViewport(0,0,::scr_width,::scr_height);

        shader.compile();
        shader.use();
        shader.set_mat4("proj", projection);
        shader.set_mat4("view", view.matrix());
        shader.set_mat4("model", model.matrix());


        if (wire_frame)
            glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        else
            glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);


        glBindVertexArray(VAO);
        glDrawElements(GL_TRIANGLES, F.size(), GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);

 
        glfwSwapBuffers(window);
        glfwPollEvents();
        sleep_by_fps(60, tic);
    }

    glDeleteVertexArrays(1, &VAO);
    glDeleteVertexArrays(1, &vao_box);
    glfwDestroyWindow(window);
    glfwTerminate();
    return EXIT_SUCCESS;
}