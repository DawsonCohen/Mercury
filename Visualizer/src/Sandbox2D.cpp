#include "Sandbox2D.h"
#include <imgui/imgui.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

using namespace EvoDevo;

void Sandbox2D::Config(VisualizerConfig config)
{
	m_Config = config;
	
	if(config.objectives.verify) {
		m_AssetManager.loadAssets(config.io.in_dir);
	} else {
		m_AssetManager.loadRandomAssets(config.visualizer.rand_count, config.robot_type);
	}

	std::cout << "Ouput directory: " << config.io.out_dir << std::endl;
	std::cout << "Input directory: " << config.io.in_dir << std::endl;

}

void Sandbox2D::OnAttach()
{
	EL_PROFILE_FUNCTION();
}

void Sandbox2D::OnDetach()
{
	EL_PROFILE_FUNCTION();
}

void Sandbox2D::OnUpdate(Elastic::Timestep ts)
{
	EL_PROFILE_FUNCTION();

	// Update
	m_CameraController.OnUpdate(ts);

	Elastic::Mesh* asset = m_AssetManager.getCurrentAsset();

	// Render
	{
		EL_PROFILE_SCOPE("Renderer Prep");
		Elastic::RenderCommand::SetClearColor({ 0.1f, 0.1f, 0.1f, 1 });
		Elastic::RenderCommand::Clear();
	}

	{
		EL_PROFILE_SCOPE("Renderer Draw");
		Elastic::Renderer3D::BeginScene(m_CameraController.GetCamera());
		Elastic::Renderer3D::DrawMesh(*asset);
		Elastic::Renderer3D::EndScene();
	}
}

void Sandbox2D::OnImGuiRender()
{
	EL_PROFILE_FUNCTION();

	ImGui::Begin("Settings");
	ImGui::ColorEdit4("Square Color", glm::value_ptr(m_Color));
	ImGui::End();
}

bool Sandbox2D::OnKeyPressed(Elastic::KeyPressedEvent& e)
{
	switch(e.GetKeyCode()) {
		case Elastic::Key::Tab:
			m_AssetManager.switchToNextAsset();
			break;
		case Elastic::Key::Up:
			break;
		case Elastic::Key::Down:
			break;
		default:
			break;
	}
	
	return false;
}

void Sandbox2D::OnEvent(Elastic::Event& e)
{
	m_CameraController.OnEvent(e);

	Elastic::EventDispatcher dispatcher(e);
	dispatcher.Dispatch<Elastic::KeyPressedEvent>(EL_BIND_EVENT_FN(Sandbox2D::OnKeyPressed));
}
