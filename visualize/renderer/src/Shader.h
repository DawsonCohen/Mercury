#ifndef __SHADER_H__
#define __SHADER_H__

#include <glad/glad.h>
#include "glm/glm.hpp"
#include <string>
#include <unordered_map>

std::string get_file_contents(const char* filename);

class Shader
{
private:
    std::string mVertexFilePath;
    std::string mFragmentFilePath;
    //caching for uniforms
    std::unordered_map<std::string, int> mUniformLocationCache;
    
public:
    unsigned int ID;
    Shader(const std::string& vertexFile, const std::string& fragmentFile);
    Shader() : ID(0) {};
    ~Shader();

    void LoadShader(const std::string& vertexFile, const std::string& fragmentFile);
    void Bind() const;
    void Unbind() const;

    // Set uniforms
    void SetUniformMatrix4fv(const std::string& name, const glm::mat4& value) ;

private:
    unsigned int CompileShader(unsigned int type, const std::string& source) const;
    int GetUniformLocation(const std::string& name);

};

#endif