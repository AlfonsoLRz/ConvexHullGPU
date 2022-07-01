#include "stdafx.h"
#include "GUI.h"

#include "Graphics/Application/Renderer.h"
#include "Interface/Fonts/font_awesome.hpp"
#include "Interface/Fonts/lato.hpp"
#include "Interface/Fonts/IconsFontAwesome5.h"

/// [Protected methods]

GUI::GUI() :
	_showRenderingSettings(false), _showSceneSettings(false), _showScreenshotSettings(false), _showAboutUs(false), _showControls(false), _currentGizmoOperation(ImGuizmo::TRANSLATE), _currentGizmoMode(ImGuizmo::WORLD)
{
	_renderer			= Renderer::getInstance();	
	_renderingParams	= Renderer::getInstance()->getRenderingParameters();
}

void GUI::createMenu()
{
	ImGuiIO& io = ImGui::GetIO();

	if (_showRenderingSettings)		showRenderingSettings();
	if (_showScreenshotSettings)	showScreenshotSettings();
	if (_showSceneSettings)			showSceneSettings();
	if (_showAboutUs)				showAboutUsWindow();
	if (_showControls)				showControls();

	if (ImGui::BeginMainMenuBar())
	{
		if (ImGui::BeginMenu(ICON_FA_COG "Settings"))
		{
			ImGui::MenuItem(ICON_FA_CUBE "Rendering", NULL, &_showRenderingSettings);
			ImGui::MenuItem(ICON_FA_IMAGE "Screenshot", NULL, &_showScreenshotSettings);
			ImGui::MenuItem(ICON_FA_TREE "Scene", NULL, &_showSceneSettings);
			ImGui::EndMenu();
		}

		if (ImGui::BeginMenu(ICON_FA_QUESTION_CIRCLE "Help"))
		{
			ImGui::MenuItem(ICON_FA_INFO "About the project", NULL, &_showAboutUs);
			ImGui::MenuItem(ICON_FA_GAMEPAD "Controls", NULL, &_showControls);
			ImGui::EndMenu();
		}

		ImGui::SameLine(io.DisplaySize.x - 125, 0);
		this->renderHelpMarker("Avoids some movements to also modify the camera parameters");
		
		ImGui::SameLine(0, 20);
		ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
		ImGui::EndMainMenuBar();
	}

	this->renderGuizmo();
}

void GUI::editTransform(ImGuizmo::OPERATION& operation, ImGuizmo::MODE& model)
{
	ImGui::NewLine();
	ImGui::SameLine(30, 0);
	if (ImGui::RadioButton("Translate", operation == ImGuizmo::TRANSLATE))
	{
		operation = ImGuizmo::TRANSLATE;
	}

	ImGui::SameLine();
	if (ImGui::RadioButton("Rotate", operation == ImGuizmo::ROTATE))
	{
		operation = ImGuizmo::ROTATE;
	}

	ImGui::SameLine();
	if (ImGui::RadioButton("Scale", operation == ImGuizmo::SCALE))
	{
		operation = ImGuizmo::SCALE;
	}

	ImGui::NewLine();
	ImGui::SameLine(30, 0);
	if (operation != ImGuizmo::SCALE)
	{
		if (ImGui::RadioButton("Local", model == ImGuizmo::LOCAL))
		{
			model = ImGuizmo::LOCAL;
		}

		ImGui::SameLine();
		if (ImGui::RadioButton("World", model == ImGuizmo::WORLD))
		{
			model = ImGuizmo::WORLD;
		}
	}
}

void GUI::leaveSpace(const unsigned numSlots)
{
	for (int i = 0; i < numSlots; ++i)
	{
		ImGui::Spacing();
	}
}

void GUI::renderHelpMarker(const char* message)
{
	ImGui::TextDisabled(ICON_FA_QUESTION);
	if (ImGui::IsItemHovered())
	{
		ImGui::BeginTooltip();
		ImGui::PushTextWrapPos(ImGui::GetFontSize() * 35.0f);
		ImGui::TextUnformatted(message);
		ImGui::PopTextWrapPos();
		ImGui::EndTooltip();
	}
}

void GUI::showAboutUsWindow()
{
	if (ImGui::Begin("About the project", &_showAboutUs))
	{
		ImGui::Text("This code belongs to a research project from University of Jaen (GGGJ group).");	
	}

	ImGui::End();
}

void GUI::showControls()
{
	if (ImGui::Begin("Scene controls", &_showControls))
	{
		ImGui::Columns(2, "ControlColumns"); // 4-ways, with border
		ImGui::Separator();
		ImGui::Text("Movement"); ImGui::NextColumn();
		ImGui::Text("Control"); ImGui::NextColumn();
		ImGui::Separator();

		const int NUM_MOVEMENTS = 14;
		const char* movement[] = { "Orbit (XZ)", "Undo Orbit (XZ)", "Orbit (Y)", "Undo Orbit (Y)", "Dolly", "Truck", "Boom", "Crane", "Reset Camera", "Take Screenshot", "Continue Animation", "Zoom +/-", "Pan", "Tilt" };
		const char* controls[] = { "X", "Ctrl + X", "Y", "Ctrl + Y", "W, S", "D, A", "Up arrow", "Down arrow", "R", "K", "I", "Scroll wheel", "Move mouse horizontally(hold button)", "Move mouse vertically (hold button)" };

		for (int i = 0; i < NUM_MOVEMENTS; i++)
		{
			ImGui::Text(movement[i]); ImGui::NextColumn();
			ImGui::Text(controls[i]); ImGui::NextColumn();
		}

		ImGui::Columns(1);
		ImGui::Separator();

	}

	ImGui::End();
}

void GUI::showRenderingSettings()
{
	if (ImGui::Begin("Rendering Settings", &_showRenderingSettings))
	{
		ImGui::ColorEdit3("Background color", &_renderingParams->_backgroundColor[0]);

		this->leaveSpace(3);

		if (ImGui::BeginTabBar("LiDARTabBar"))
		{
			if (ImGui::BeginTabItem("General settings"))
			{
				this->leaveSpace(1);

				ImGui::Separator();
				ImGui::Text(ICON_FA_LIGHTBULB "Lighting");

				ImGui::SliderFloat("Scattering", &_renderingParams->_materialScattering, 0.0f, 10.0f);

				this->leaveSpace(2);

				ImGui::Separator();
				ImGui::Text(ICON_FA_TREE "Scenario");

				ImGui::Checkbox("Render scenario", &_renderingParams->_showTriangleMesh);

				{
					ImGui::Spacing();

					ImGui::NewLine();
					ImGui::SameLine(30, 0);
					ImGui::Checkbox("Screen Space Ambient Occlusion", &_renderingParams->_ambientOcclusion);

					const char* visualizationTitles[] = { "Points", "Lines", "Triangles", "All" };
					ImGui::NewLine();
					ImGui::SameLine(30, 0);
					ImGui::Combo("Visualization", &_renderingParams->_visualizationMode, visualizationTitles, IM_ARRAYSIZE(visualizationTitles));

					ImGui::Spacing();

					this->editTransform(_currentGizmoOperation, _currentGizmoMode);
				}

				ImGui::EndTabItem();
			}

			if (ImGui::BeginTabItem("Data Structures"))
			{
				this->leaveSpace(1);

				ImGui::Checkbox("Render BVH", &_renderingParams->_showBVH);

				{
					this->leaveSpace(1);
					ImGui::NewLine(); ImGui::SameLine(0, 22);
					ImGui::ColorEdit3("BVH color", &_renderingParams->_bvhWireframeColor[0]);
					ImGui::NewLine(); ImGui::SameLine(0, 22);
					ImGui::SliderFloat("BVH nodes", &_renderingParams->_bvhNodesPercentage, 0.0f, 1.0f);
					this->leaveSpace(2);
				}

				this->leaveSpace(1);

				ImGui::EndTabItem();
			}

			if (ImGui::BeginTabItem("Point Cloud"))
			{
				this->leaveSpace(1);

				ImGui::SliderFloat("Point Size", &_renderingParams->_scenePointSize, 0.1f, 50.0f);
				ImGui::ColorEdit3("Point Cloud Color", &_renderingParams->_scenePointCloudColor[0]);

				ImGui::EndTabItem();
			}

			if (ImGui::BeginTabItem("Wireframe"))
			{
				this->leaveSpace(1);

				ImGui::ColorEdit3("Wireframe Color", &_renderingParams->_wireframeColor[0]);
				ImGui::Checkbox("Render Scene Normals", &_renderingParams->_showVertexNormal);
				ImGui::SameLine(0, 20); ImGui::PushItemWidth(150.0f);  ImGui::SliderFloat("Normal Length", &_renderingParams->_normalLength, .1f, 10.0f); ImGui::PopItemWidth();

				ImGui::EndTabItem();
			}

			ImGui::EndTabBar();
		}
	}

	ImGui::End();
}

void GUI::showSceneSettings()
{
	if (_modelComponents.empty()) _modelComponents = _renderer->getCurrentScene()->getModelComponents();
	
	ImGui::SetNextWindowSize(ImVec2(480, 440), ImGuiCond_FirstUseEver);

	if (ImGui::Begin("Scene Models", &_showSceneSettings, ImGuiWindowFlags_None))
	{
		this->leaveSpace(4);

		// Left
		static int modelCompSelected = 0;

		ImGui::BeginChild("Model Components", ImVec2(200, 0), true);

		for (int i = 0; i < _modelComponents.size(); ++i)
		{
			if (ImGui::Selectable(_modelComponents[i]->_name.c_str(), modelCompSelected == i))
				modelCompSelected = i;
		}

		ImGui::EndChild();

		ImGui::SameLine();

		// Right
		ImGui::BeginGroup();
		ImGui::BeginChild("Model Component View", ImVec2(0, -ImGui::GetFrameHeightWithSpacing()));		// Leave room for 1 line below us

		ImGui::Text(_modelComponents[modelCompSelected]->_name.c_str());
		ImGui::Separator();

		this->leaveSpace(1);
				
		ImGui::Checkbox("Enabled", &_modelComponents[modelCompSelected]->_enabled);

		ImGui::EndChild();
		ImGui::EndGroup();
	}

	ImGui::End();
}

void GUI::showScreenshotSettings()
{
	if (ImGui::Begin("Screenshot Settings", &_showScreenshotSettings, ImGuiWindowFlags_AlwaysAutoResize))
	{
		ImGui::SliderFloat("Size multiplier", &_renderingParams->_screenshotMultiplier, 1.0f, 10.0f);
		ImGui::InputText("Filename", _renderingParams->_screenshotFilenameBuffer, IM_ARRAYSIZE(_renderingParams->_screenshotFilenameBuffer));

		this->leaveSpace(2);

		ImGui::PushID(0);
		ImGui::PushStyleColor(ImGuiCol_Button, (ImVec4)ImColor::HSV(1 / 7.0f, 0.6f, 0.6f));
		ImGui::PushStyleColor(ImGuiCol_ButtonHovered, (ImVec4)ImColor::HSV(1 / 7.0f, 0.7f, 0.7f));
		ImGui::PushStyleColor(ImGuiCol_ButtonActive, (ImVec4)ImColor::HSV(1 / 7.0f, 0.8f, 0.8f));

		if (ImGui::Button("Take screenshot"))
		{
			std::string filename = _renderingParams->_screenshotFilenameBuffer;

			if (filename.empty())
			{
				filename = "Screenshot.png";
			}
			else if (filename.find(".png") == std::string::npos)
			{
				filename += ".png";
			}

			Renderer::getInstance()->getScreenshot(filename);
		}

		ImGui::PopStyleColor(3);
		ImGui::PopID();
	}

	ImGui::End();
}

GUI::~GUI()
{	
	ImGui::DestroyContext();
}

/// [Public methods]

void GUI::initialize(GLFWwindow* window, const int openGLMinorVersion)
{
	const std::string openGLVersion = "#version 4" + std::to_string(openGLMinorVersion) + "0 core";

	IMGUI_CHECKVERSION();
	ImGui::CreateContext();

	this->loadImGUIStyle();
	
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init(openGLVersion.c_str());
}

void GUI::render()
{
	bool show_demo_window = true;

	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
	ImGuizmo::BeginFrame();

	this->createMenu();

	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

// ---------------- IMGUI ------------------

void GUI::loadImGUIStyle()
{
	ImGui::StyleColorsDark();

	this->loadFonts();
}

void GUI::renderGuizmo()
{
	Scene* scene = _renderer->getCurrentScene();
	const GLint modelComponentSelected = scene->getGroupSelected();

	if (modelComponentSelected >= 0)
	{
		static ImGuizmo::OPERATION mCurrentGizmoOperation(ImGuizmo::TRANSLATE);
		static ImGuizmo::MODE mCurrentGizmoMode(ImGuizmo::WORLD);

		if (ImGui::IsKeyPressed(90))
		{
			mCurrentGizmoOperation = ImGuizmo::TRANSLATE;
		}

		if (ImGui::IsKeyPressed(69))
		{
			mCurrentGizmoOperation = ImGuizmo::ROTATE;
		}

		if (ImGui::IsKeyPressed(82))
		{
			mCurrentGizmoOperation = ImGuizmo::SCALE;
		}

		// Render Guizmo outside any window
		const mat4 viewMatrix = _renderer->getCurrentScene()->getCameraManager()->getActiveCamera()->getViewMatrix();
		const mat4 projectionMatrix = _renderer->getCurrentScene()->getCameraManager()->getActiveCamera()->getProjectionMatrix();
		mat4 modelMatrix = scene->getGroupMatrix(modelComponentSelected);
		
		ImGuiIO& io = ImGui::GetIO();
		ImGuizmo::SetRect(0, 0, io.DisplaySize.x, io.DisplaySize.y);
		ImGuizmo::Manipulate(&(viewMatrix[0][0]), &(projectionMatrix[0][0]), mCurrentGizmoOperation, mCurrentGizmoMode, &(modelMatrix[0][0]), nullptr, nullptr);

		scene->setGroupMatrix(modelComponentSelected, modelMatrix);
	}
}

void GUI::loadFonts()
{
	ImFontConfig cfg;
	ImGuiIO& io = ImGui::GetIO();
	
	std::copy_n("Lato", 5, cfg.Name);
	io.Fonts->AddFontFromMemoryCompressedBase85TTF(lato_compressed_data_base85, 15.0f, &cfg);

	static const ImWchar icons_ranges[] = { ICON_MIN_FA, ICON_MAX_FA, 0 };
	cfg.MergeMode = true;
	cfg.PixelSnapH = true;
	cfg.GlyphMinAdvanceX = 20.0f;
	cfg.GlyphMaxAdvanceX = 20.0f;
	std::copy_n("FontAwesome", 12, cfg.Name);

	io.Fonts->AddFontFromFileTTF("Assets/Fonts/fa-regular-400.ttf", 13.0f, &cfg, icons_ranges);
	io.Fonts->AddFontFromFileTTF("Assets/Fonts/fa-solid-900.ttf", 13.0f, &cfg, icons_ranges);
}