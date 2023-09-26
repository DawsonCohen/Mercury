#include "Shader.h"
#include "Renderer.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <cerrno>
 
std::string get_file_contents(const char* filename) {
    std::ifstream in(filename, std::ios::binary);
	if (in)
	{
		std::string contents;
		in.seekg(0, std::ios::end);
		contents.resize(in.tellg());
		in.seekg(0, std::ios::beg);
		in.read(&contents[0], contents.size());
		in.close();
		return(contents);
	}
	throw(errno);
}

unsigned int Shader::CompileShader(unsigned int type, const std::string& source) const {
    const char* src = source.c_str();

    unsigned int id = glCreateShader(type);
    glShaderSource(id, 1, &src, NULL);
    glCompileShader(id);

    // check for shader compile errors
    int success;
    char infoLog[512];
    glGetShaderiv(id, GL_COMPILE_STATUS, &success);
    if (!success)
    {
        glGetShaderInfoLog(id, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::" << (type == GL_VERTEX_SHADER ? "VERTEX" : "FRAGMENT") << "::COMPILATION_FAILED\n" << infoLog << std::endl;

        glDeleteShader(id);
        return 0;
    }

    return id;
}

Shader::Shader(const char* vertexFile, const char* fragmentFile) :
    mVertexFilePath(vertexFile), mFragmentFilePath(fragmentFile) {
    std::string vertexSource = get_file_contents(vertexFile);
    std::string fragmentSource = get_file_contents(fragmentFile);

    unsigned int vertexShader = CompileShader(GL_VERTEX_SHADER, vertexSource);
    unsigned int fragmentShader = CompileShader(GL_FRAGMENT_SHADER, fragmentSource);

    // link shaders
    ID = glCreateProgram();
    GLCall(glAttachShader(ID, vertexShader));
    GLCall(glAttachShader(ID, fragmentShader));
    GLCall(glLinkProgram(ID));
    GLCall(glValidateProgram(ID));

    // check for shader compile errors
    int success;
    char infoLog[512];
    glGetShaderiv(ID, GL_LINK_STATUS, &success);
    if (!success)
    {
        glGetProgramInfoLog(ID, 512, NULL, infoLog);
        std::cout << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
    }

    GLCall(glDeleteShader(vertexShader));
    GLCall(glDeleteShader(fragmentShader));
}

Shader::~Shader() {
    GLCall(glDeleteProgram(ID));
}

void Shader::Bind() const {
    GLCall(glUseProgram(ID));
}

void Shader::Unbind() const {
    GLCall(glUseProgram(0));
}

void Shader::SetUniformMatrix4fv(const std::string& name, const glm::mat4& value) {
	// glUniformMatrix4fv(glGetUniformLocation(shader.ID, uniform), 1, GL_FALSE, glm::value_ptr(cameraMatrix));
    GLCall(glUniformMatrix4fv(GetUniformLocation(name), 1, GL_FALSE, glm::value_ptr(value)));
}

int Shader::GetUniformLocation(const std::string& name) {
    if(mUniformLocationCache.find(name) != mUniformLocationCache.end())
        return mUniformLocationCache[name];

    GLCall(int location = glGetUniformLocation(ID, name.c_str()));
    if(location == -1) 
        std::cout << "Warning unfirom '" << name << "' does not exist" << std::endl;
    
    mUniformLocationCache[name] = location;
    return location;
}
