#include <Eigen/Core>

#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <igl/readOBJ.h>
#include <igl/ortho.h>
#include <igl/opengl/vertex_array.h>
#include <igl/opengl/report_gl_error.h>

#include <iostream>
#include <vector>
#include <string>
#include <cmath>

#include "src/time_utils.h"
#include "src/print_opengl_info.h"
#include "src/shader.h"

using namespace Eigen;
using namespace std;

int scr_width  = 500;
int scr_height = 400;

string filename;
string data_dir   = "../data/";
string shader_dir = "../src/shaders/";

const auto getfilepath = [](const string& name, const string& ext){ 
    return data_dir + name + "." + ext; 
};

MatrixXd V;
MatrixXi F;

GLuint vao, vbo, ebo;

bool wire_frame = false;
bool mouse_down = false;
Affine3f view = Affine3f::Identity() * Translation3f(Vector3f(0,0,-10));
Matrix4f proj = Matrix4f::Identity();


int main(int argc, char* argv[])
{
    filename = "small";
    if (argc > 1) { filename = string(argv[1]); }
    igl::readOBJ(getfilepath(filename, "obj"), V, F);

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
    )";
    glfwSetWindowSizeCallback(window, [](GLFWwindow* window, int width, int height){
        ::scr_width = width; ::scr_height = height;
        float near, far, ratio, top;
        near = 0.01; far  = 2.; top  = 1.;
        ratio = float(width) / float(height);
        igl::ortho(-top*ratio, top*ratio, -top, top, near, far, ::proj);
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
            default:
                std::cout<<"Unrecognized key: "<<static_cast<char>(codepoint)<<'\n';
                break;
        }
    });
    glfwSetMouseButtonCallback(window, [](GLFWwindow* window, int button, int action, int mods) {
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
        view.matrix()(2,3) = min(max(view.matrix()(2,3)+(float)yoffset,-100.0f),-2.0f);
    });

    
    auto vspaths = vector<string>({shader_dir+"simple.vs"});
    auto fspaths = vector<string>({shader_dir+"simple.fs"});
    auto shader = Shader(vspaths, fspaths);
    shader.compile();

    // igl::opengl::vertex_array(V, F, vao, vbo, ebo);
    // igl::opengl::report_gl_error("vertex_array");

    // std::cout<<"V: \n"<<V.block(0,0,V.rows(),V.cols())<<'\n';
    // std::cout<<"F: \n"<<F.block(0,0,F.rows(),F.cols())<<'\n';


    float vertices[] = {
         0.5f,  0.5f, 0.0f,  // top right
         0.5f, -0.5f, 0.0f,  // bottom right
        -0.5f, -0.5f, 0.0f,  // bottom left
        -0.5f,  0.5f, 0.0f   // top left 
    };
    unsigned int indices[] = {  // note that we start from 0!
        0, 1, 3,  // first Triangle
        1, 2, 3   // second Triangle
    };

    unsigned int VBO, VAO, EBO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    // bind the Vertex Array Object first, then bind and set vertex buffer(s), and then configure vertex attributes(s).
    glBindVertexArray(VAO);

    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(indices), indices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0); 



    glEnable(GL_DEPTH_TEST);  
    glEnable(GL_CULL_FACE);

    while (!glfwWindowShouldClose(window)) 
    {
        double tic = get_seconds();
        glClearColor(0.2, 0.2, 0.2, 1.);
        glClear(GL_COLOR_BUFFER_BIT);

        shader.compile();
        // shader.use();
        glUseProgram(shader.prog_id);
        shader.set_mat4("proj", ::proj);
        shader.set_mat4("view", ::view.matrix());

        glBindVertexArray(VAO);
        // glDrawElements(GL_TRIANGLES, F.size(), GL_UNSIGNED_INT, 0);
        glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);
        glBindVertexArray(0);
 
        glfwSwapBuffers(window);
        glfwPollEvents();
        sleep_by_fps(60, tic);
    }

    glDeleteVertexArrays(1, &VAO);
    glfwDestroyWindow(window);
    glfwTerminate();
    return EXIT_SUCCESS;
}