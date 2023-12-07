#pragma once

#include "Elastic.h"

class Sandbox2D : public Elastic::Layer
{
public:
	Sandbox2D();
	virtual ~Sandbox2D() = default;

	virtual void OnAttach() override;
	virtual void OnDetach() override;

	void OnUpdate(Elastic::Timestep ts) override;
	virtual void OnImGuiRender() override;
	void OnEvent(Elastic::Event& e) override;
private:
	Elastic::CameraController m_CameraController;
	
	// Temp
	Elastic::Ref<Elastic::VertexArray> m_SquareVA;
	Elastic::Ref<Elastic::Shader> m_FlatColorShader;

	glm::vec4 m_Color = { 0.2f, 0.3f, 0.8f, 1.0f };
};