#include "Visualizer.h"
#include <imgui/imgui.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

using namespace EvoDevo;

Visualizer::Visualizer(std::string config_file) :
	Layer("Visualizer"), m_CameraController(), m_Color({ 0.2f, 0.3f, 0.8f, 1.0f })
{
	m_Config = Util::ReadConfigFile(config_file);
	Config(m_Config);
	NNRobot::Configure(m_Config.nnrobot);

	m_CameraController.SetDistance(20.0f);
	m_CameraController.SetPosition({0.0f, 4.0f, 20.0f});
}

void Visualizer::Config(VisualizerConfig config)
{
	m_Config = config;

	switch(config.simulator.env_type) {
		case ENVIRONMENT_LAND:
			m_Environment = EnvironmentLand;
			break;
		case ENVIRONMENT_WATER:
		default:
			m_Environment = EnvironmentWater;
			break;
	}

	m_deltaT = config.simulator.time_step;
	
	m_Sim.Initialize(m_Config.simulator);
	
	if(config.objectives.verify) {
		m_AssetManager.loadAssets(config.io.in_dir);
	} else {
		m_AssetManager.loadRandomAssets(config.visualizer.rand_count, config.robot_type);
	}

	std::cout << "Ouput directory: " << config.io.out_dir << std::endl;
	std::cout << "Input directory: " << config.io.in_dir << std::endl;

	RobotModel* R = m_AssetManager.getCurrentAsset();
	m_ElementTracker = m_Sim.SetElement(*R);
}

void Visualizer::OnAttach()
{
	EL_PROFILE_FUNCTION();
}

void Visualizer::OnDetach()
{
	EL_PROFILE_FUNCTION();
}

void Visualizer::OnUpdate(Elastic::Timestep ts)
{
	EL_PROFILE_FUNCTION();

	// Update
	m_CameraController.OnUpdate(ts);

	RobotModel* asset = m_AssetManager.getCurrentAsset();
	// Render
	{
		EL_PROFILE_SCOPE("Renderer Prep");
		Elastic::RenderCommand::SetClearColor({ 0.1f, 0.1f, 0.1f, 1 });
		Elastic::RenderCommand::Clear();
	}


	// Animate
	{
		m_Sim.Simulate(m_SimSpeed * m_TimeStep,false);
		Element Relement = m_Sim.Collect(m_ElementTracker);
		asset->Update(Relement, m_ShowFaces);
		asset->updateFitness();
		m_ForceMesh.Update(Relement, m_Environment);

		if(!headless) {
			std::cout << "Distance: " << (asset->getBaseCOM() - asset->getCOM()).norm() << std::endl;
			while ((Elastic::Time::GetTime() - ts.GetSeconds()) < m_TimeStep) {
				continue;
			}
		}
	}

	{
		EL_PROFILE_SCOPE("Renderer Draw");
		Elastic::Renderer3D::BeginScene(m_CameraController.GetCamera());
		Elastic::Renderer3D::DrawMesh(*asset);
		if(m_ShowForces) Elastic::Renderer3D::DrawMesh(m_ForceMesh);
		Elastic::Renderer3D::EndScene();
	}
}

void Visualizer::OnImGuiRender()
{
	EL_PROFILE_FUNCTION();

	ImGui::Begin("Settings");
	ImGui::SliderFloat("drag", &m_Environment.drag, 0.0f, 2000.0f);
	ImGui::SliderFloat("deltaT", &m_deltaT, 0.00001f, 0.005f, "%.7f");
	ImGui::End();

	m_Sim.SetEnvironment(m_Environment);
	m_Sim.setTimeStep(m_deltaT);
}

bool Visualizer::OnKeyPressed(Elastic::KeyPressedEvent& e)
{
	switch(e.GetKeyCode()) {
		case Elastic::Key::Tab:
			HandleAssetChange();
			break;
		case Elastic::Key::Up:
			m_SimSpeed = std::min(2.0f, m_SimSpeed * 2.0f);
			EL_CORE_INFO("Sim speed: {0}", m_SimSpeed);
			break;
		case Elastic::Key::Down:
			m_SimSpeed = std::max((float) std::pow(2.0f,-32), m_SimSpeed / 2.0f);
			EL_CORE_INFO("Sim speed: {0}", m_SimSpeed);
			break;
		case Elastic::Key::F:
			m_ShowFaces = !m_ShowFaces;
			break;
		case Elastic::Key::V:
			m_ShowForces = !m_ShowForces;
			break;
		default:
			break;
	}
	
	return false;
}

void Visualizer::HandleAssetChange()
{
	m_AssetManager.switchToNextAsset();
	RobotModel* R = m_AssetManager.getCurrentAsset();
	m_ElementTracker = m_Sim.SetElement(*R);
	m_Sim.Reset();
}

void Visualizer::OnEvent(Elastic::Event& e)
{
	m_CameraController.OnEvent(e);

	Elastic::EventDispatcher dispatcher(e);
	dispatcher.Dispatch<Elastic::KeyPressedEvent>(EL_BIND_EVENT_FN(Visualizer::OnKeyPressed));
}
