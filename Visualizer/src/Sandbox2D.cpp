#include "Sandbox2D.h"
#include <imgui/imgui.h>

#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

Sandbox2D::Sandbox2D()
	: Layer("EvoDevo"), m_CameraController(), m_Color({ 0.2f, 0.3f, 0.8f, 1.0f })
{

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

	// Render
	{
		EL_PROFILE_SCOPE("Renderer Prep");
		Elastic::RenderCommand::SetClearColor({ 0.1f, 0.1f, 0.1f, 1 });
		Elastic::RenderCommand::Clear();
	}

	{
		static float rotation = 0.0f;
		rotation += ts * 50.0f;

		EL_PROFILE_SCOPE("Renderer Draw");
		Elastic::Renderer3D::BeginScene(m_CameraController.GetCamera());
		Elastic::Renderer3D::DrawTriangle({ -1.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, m_Color);
		Elastic::Renderer3D::DrawLine({ -1.0f, 0.0f, 0.0f }, { 1.0f, 0.0f, 0.0f }, m_Color);
		Elastic::Renderer3D::DrawLine({ -1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, m_Color);
		Elastic::Renderer3D::DrawLine({  1.0f, 0.0f, 0.0f }, { 0.0f, 1.0f, 0.0f }, m_Color);
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

void Sandbox2D::OnEvent(Elastic::Event& e)
{
	m_CameraController.OnEvent(e);
}
