#pragma once
#include <Eigen/Core>
#include "glad/glad.h"

#include <string>
#include <vector>

class Shader {
    using shader_paths = std::vector<std::string>;
public:
    Shader(const shader_paths& vertex_shader_paths, const shader_paths& fragment_shader_paths);
public:
    void use();
    void compile();

    void set_bool(const std::string& name, bool value) const;
    void set_int (const std::string& name, int value) const;
    void set_vec3(const std::string& name, Eigen::Matrix<float, 3, 1, Eigen::ColMajor> value) const;
    void set_vec3(const std::string& name, Eigen::Matrix<float, 1, 3, Eigen::RowMajor> value) const;
    void set_mat4(const std::string& name, Eigen::Matrix<float, 4, 4, Eigen::RowMajor> value) const;
    void set_mat4(const std::string& name, Eigen::Matrix<float, 4, 4, Eigen::ColMajor> value) const;
private:
    bool any_changed(const std::vector<std::string>& paths);
public:
    GLuint prog_id;
    double time_of_prev_compilation;
    shader_paths vertex_shader_paths;
    shader_paths fragment_shader_paths;
};
