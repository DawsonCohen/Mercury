#pragma once

#include "Elastic.h"
#include "EvoDevo.h"
#include "Util/visualizer_config.h"
#include "AssetManager.h"
#include "Models/force_mesh.h"

class Visualizer : public Elastic::Layer
{
public:
	Visualizer(std::string config_file);
	virtual ~Visualizer() = default;

	virtual void OnAttach() override;
	virtual void OnDetach() override;

	void Config(VisualizerConfig config);

	void OnUpdate(Elastic::Timestep ts) override;
	virtual void OnImGuiRender() override;
	void OnEvent(Elastic::Event& e) override;

private:
	bool OnKeyPressed(Elastic::KeyPressedEvent& e);
	void HandleAssetChange();

private:
	Elastic::CameraController m_CameraController;
	
	// Temp
	Elastic::Ref<Elastic::VertexArray> m_SquareVA;
	Elastic::Ref<Elastic::Shader> m_FlatColorShader;

    bool headless = false;
	float m_SimSpeed = 1.0f;
	float m_FrameRate = 60.0f;
	float m_TimeStep = 1.0f / m_FrameRate;

	RobotModel* m_CurrentRobot;

	float m_deltaT = 0.001f;
	EvoDevo::Environment m_Environment;
	RobotModel* m_RobotModel;
	ForceMesh m_ForceMesh;
	bool m_ShowFaces = true;
	bool m_ShowForces = false;
	float m_FaceAlpha = 0.8;
	
    VisualizerConfig m_Config;
    AssetManager m_AssetManager;
    EvoDevo::Simulator m_Sim;
    EvoDevo::ElementTracker m_ElementTracker;

	glm::vec4 m_Color = { 0.2f, 0.3f, 0.8f, 1.0f };
};