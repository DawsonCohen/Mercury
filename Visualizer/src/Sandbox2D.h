#pragma once

#include "Elastic.h"
#include "Simulator.h"
#include "Util/visualizer_config.h"
#include "AssetManager.h"

class Sandbox2D : public Elastic::Layer
{
public:
	Sandbox2D();
	Sandbox2D(std::string config_file);
	virtual ~Sandbox2D() = default;

	virtual void OnAttach() override;
	virtual void OnDetach() override;

	void Config(VisualizerConfig config);

	void OnUpdate(Elastic::Timestep ts) override;
	virtual void OnImGuiRender() override;
	void OnEvent(Elastic::Event& e) override;

private:
	bool OnKeyPressed(Elastic::KeyPressedEvent& e);

private:
	Elastic::CameraController m_CameraController;
	
	// Temp
	Elastic::Ref<Elastic::VertexArray> m_SquareVA;
	Elastic::Ref<Elastic::Shader> m_FlatColorShader;

    bool headless;
    VisualizerConfig m_Config;
    AssetManager m_AssetManager;
    EvoDevo::Simulator m_Sim;

	glm::vec4 m_Color = { 0.2f, 0.3f, 0.8f, 1.0f };
};