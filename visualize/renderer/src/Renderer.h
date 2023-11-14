#ifndef __RENDERER_H__
#define __RENDERER_H__

#include <glad/glad.h>
#include <csignal>
#include "Camera.h"
#include "Model.h"

#include <opencv2/opencv.hpp>

#define ASSERT(x) if (!(x)) std::raise(SIGINT);
#define GLCall(x) GLClearError();\
	x;\
	ASSERT(GLLogCall(#x,__FILE__,__LINE__))

void GLClearError();
bool GLLogCall(const char* function, const char* file, int line);

class Renderer {
public:
	Renderer(const std::string& vertexShaderPath, const std::string& fragmentShaderPath);
	Renderer() {}

	void Clear() const;
	void Initialize();

	void Render(const Camera& camera, Model* model);

private:
	cv::VideoWriter video;
	Shader shader;
};

#endif