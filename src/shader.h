#pragma once

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
    {
        compile();
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
    // shader program id
    GLuint prog_id;
    // time to previous compilation 
    double time_of_prev_compilation;
    // paths to {vertex, fragment} shaders
    shader_paths vertex_shader_paths;
    shader_paths fragment_shader_paths;
};
