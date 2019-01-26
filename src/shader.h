#pragma once
#include <Eigen/Core>

#include <string>
#include <vector>
#include <iostream>

#include "time_utils.h"
#include "last_modification_time.h"
#include "create_shader_program_from_files.h"

class Shader {
    using shader_paths = std::vector<std::string>;
public:
    Shader(const shader_paths& vertex_shader_paths,
           const shader_paths& fragment_shader_paths)
           : prog_id(0),
             time_of_prev_compilation(0),
             vertex_shader_paths(vertex_shader_paths),
             fragment_shader_paths(fragment_shader_paths)
        {}

    void use() {
        glUseProgram(prog_id);
    }
 
    void compile() {
        shader_paths empty;
        if (any_changed(vertex_shader_paths) || any_changed(fragment_shader_paths)) {
            std::cout<<"--------------------------------------------------------------------\n";
            time_of_prev_compilation = get_seconds();
            auto success = create_shader_program_from_files(
                vertex_shader_paths, empty, empty, fragment_shader_paths,
                prog_id
            );
            if (!success) {
                glDeleteProgram(prog_id);
                prog_id = 0;
            } else {
                std::cout << "Successfully compiled shaders\n";
            }
        }
    }


    void set_bool(const std::string& name, bool value) const {
        glUniform1i(glGetUniformLocation(prog_id, name.c_str()), value);
    }
    void set_int(const std::string& name, int value) const {
        glUniform1i(glGetUniformLocation(prog_id, name.c_str()), value);
    }
    void set_vec3(const std::string& name,
        Eigen::Matrix<float, 3, 1, Eigen::ColMajor> value) const {
        glUniform3fv(glGetUniformLocation(prog_id, name.c_str()), 1, value.data());
    }
    void set_vec3(const std::string& name,
        Eigen::Matrix<float, 1, 3, Eigen::RowMajor> value) const {
        glUniform3fv(glGetUniformLocation(prog_id, name.c_str()), 1, value.data());
    }
    void set_mat4(const std::string& name,
        Eigen::Matrix<float, 4, 4, Eigen::RowMajor> value) const {
        glUniformMatrix4fv(glGetUniformLocation(prog_id, name.c_str()), 1,  GL_TRUE, value.data());
    }
    void set_mat4(const std::string& name,
        Eigen::Matrix<float, 4, 4, Eigen::ColMajor> value) const {
        glUniformMatrix4fv(glGetUniformLocation(prog_id, name.c_str()), 1, GL_FALSE, value.data());
    }

private:

    bool any_changed(const std::vector<std::string>& paths) {
        for (auto&& path : paths) {
            if (last_modification_time(path) > time_of_prev_compilation) {
                std::cout<<path<<" has changed since last compilation\n";
                return true;
            }
        }
        return false;
    }

public:
    GLuint prog_id;
    double time_of_prev_compilation;
    shader_paths vertex_shader_paths;
    shader_paths fragment_shader_paths;
};
