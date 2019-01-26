#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <igl/opengl/report_gl_error.h>

#include <iostream>
#include <vector>
#include <string>

#include "src/time_utils.h"
#include "src/print_opengl_info.h"
#include "src/shader.h"

using namespace std;

const unsigned int scr_width  = 500;
const unsigned int scr_height = 400;

string shader_dir = "../src/shaders/";

void processInput(GLFWwindow *window)
{
    if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);
}

int main(int argc, char* argv[])
{
    // init glfw

    if (!glfwInit()) { std::cerr<<"Could not initialize glfw\n"; return -1; }
    glfwSetErrorCallback([](int err, const char* msg) { std::cerr<<msg<<'\n'; });
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    // create glfw window 

    GLFWwindow* window = glfwCreateWindow(scr_width, scr_height, "gltest", NULL, NULL);
    if (window == NULL) { 
        std::cerr<<"Failed to create GLFW window"<<'\n';
        glfwTerminate(); 
        return EXIT_FAILURE; }
    glfwMakeContextCurrent(window);
    glfwSetInputMode(window,GLFW_CURSOR,GLFW_CURSOR_NORMAL);

    // load opengl and extensions

    if (!gladLoadGLLoader((GLADloadproc) glfwGetProcAddress)) {
        std::cerr<<"Failed to load OpenGL and its extensions\n";
        return EXIT_FAILURE; }

    print_opengl_info(window);
    igl::opengl::report_gl_error("init");
    
    
    auto vspaths = vector<string>({shader_dir+"simple.vs"});
    auto fspaths = vector<string>({shader_dir+"simple.fs"});
    auto shader = Shader(vspaths, fspaths);


    float vertices[] = {
        -0.5f, -0.5f, 0.0f, // left  
         0.5f, -0.5f, 0.0f, // right 
         0.0f,  0.5f, 0.0f  // top   
    }; 


    unsigned int VBO, VAO;
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    glBindBuffer(GL_ARRAY_BUFFER, 0); 
    glBindVertexArray(0); 


    // render loop 

    while (!glfwWindowShouldClose(window)) 
    {
        double tic = get_seconds();

        processInput(window);

        shader.compile();

        glClearColor(0.2f, 0.3f, 0.3f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        glUseProgram(shader.prog_id);
        glBindVertexArray(VAO);
        glDrawArrays(GL_TRIANGLES, 0, 3);
 
        glfwSwapBuffers(window);
        glfwPollEvents();
        sleep_by_fps(60, tic);
    }

    glfwDestroyWindow(window);
    glfwTerminate();
    return EXIT_SUCCESS;
}