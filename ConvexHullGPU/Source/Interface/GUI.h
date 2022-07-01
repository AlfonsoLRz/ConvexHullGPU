#pragma once

#include "imgui/imgui.h"
#include "imgui/examples/imgui_impl_glfw.h"
#include "imgui/examples/imgui_impl_opengl3.h"

#include "Geometry/Animation/CatmullRom.h"
#include "Graphics/Application/Renderer.h"
#include "Graphics/Application/RenderingParameters.h"
#include "imgizmo/ImGuizmo.h"
#include "Utilities/Singleton.h"

typedef std::vector<Model3D::ModelComponent*> ModelComponentBuffer;

/**
*	@file GUI.h
*	@authors Alfonso López Ruiz (alr00048@red.ujaen.es)
*	@date 08/05/2019
*/

/**
*	@brief Wrapper for GUI graphics which allows the user to interact with the scene.
*/
class GUI: public Singleton<GUI>
{
	friend class Singleton<GUI>;

protected:
	ModelComponentBuffer			_modelComponents;					//!< Model component active on the scene
	Renderer*						_renderer;							//!< Access to current scene
	RenderingParameters*			_renderingParams;					//!< Reference to rendering parameters

	// GUI state
	bool							_showAboutUs;						//!< About us window
	bool							_showControls;						//!< Shows application controls
	bool							_showRenderingSettings;				//!< Displays a window which allows the user to modify the rendering parameters
	bool							_showSceneSettings;					//!< Displays a window with all the model components and their variables
	bool							_showScreenshotSettings;			//!< Shows a window which allows to take an screenshot at any size

	// ImGuizmo
	ImGuizmo::OPERATION				_currentGizmoOperation;
	ImGuizmo::MODE					_currentGizmoMode;

protected:
	/**
	*	@brief Constructor of GUI context provided by a graphics library (Dear ImGui).
	*/
	GUI();
	
	/**
	*	@brief Creates the navbar.
	*/
	void createMenu();

	/**
	*	@brief  
	*/
	void editTransform(ImGuizmo::OPERATION& operation, ImGuizmo::MODE& mode);

	/**
	*	@brief Calls ImGui::Spacing() for n times in a clean way.
	*/
	static void leaveSpace(const unsigned numSlots);

	/**
	*	@brief  
	*/
	void loadFonts();

	/**
	*	@brief  
	*/
	void loadImGUIStyle();

	/**
	*	@brief  
	*/
	void renderGuizmo();

	/**
	*	@brief Renders a help icon with a message.
	*/
	static void renderHelpMarker(const char* message);

	/**
	*	@brief Shows a window where some information about the project is displayed.
	*/
	void showAboutUsWindow();

	/**
	*	@brief Displays a table with the application controls (mouse, keyboard, etc).
	*/
	void showControls();

	/**
	*	@brief Shows a window with general rendering configuration.
	*/
	void showRenderingSettings();

	/**
	*	@brief Shows a list of models in the scene which can be modified.
	*/
	void showSceneSettings();

	/**
	*	@brief Shows a window with framebuffer and screenshot configuration.
	*/
	void showScreenshotSettings();

public:
	/**
	*	@brief Destructor.
	*/
	virtual ~GUI();

	/**
	*	@brief Initializes the context provided by an interface library.
	*	@param window Window provided by GLFW.
	*	@param openGLMinorVersion Minor version of OpenGL 4.
	*/
	void initialize(GLFWwindow* window, const int openGLMinorVersion);

	/**
	*	@brief Renders the interface components.
	*/
	void render();

	// ------- Getters --------

	/**
	*	@return  
	*/
	bool isMouseActive() { return ImGui::GetIO().WantCaptureMouse; }
};

