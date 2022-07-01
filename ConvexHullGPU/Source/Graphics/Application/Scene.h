#pragma once

#include "Graphics/Application/CameraManager.h"
#include "Graphics/Application/MaterialList.h"
#include "Graphics/Application/RenderingParameters.h"
#include "Graphics/Core/Camera.h"
#include "Graphics/Core/Group3D.h"
#include "Graphics/Core/Light.h"
#include "Graphics/Core/Model3D.h"
#include "Graphics/Core/PlanarSurface.h"
#include "Graphics/Core/ShaderProgram.h"
#include "Graphics/Core/SSAOFBO.h"
#include "Graphics/Core/TriangleSet.h"
#include "Interface/Window.h"

typedef std::vector<Group3D::StaticGPUData*> GPUDataBuffer;

/**
*	@file Scene.h
*	@authors Alfonso López Ruiz (alr00048@red.ujaen.es)
*	@date 07/12/2019
*/

/**
*	@brief Base class for any drawable scene.
*/
class Scene
{
protected:
	// [General scene]
	std::unique_ptr<CameraManager>		_cameraManager;					//!< Contains the cameras of this scene
	std::vector<std::unique_ptr<Light>> _lights;						//!< Set of lights for the scene

	// [Groups]
	GLint								_modelCompSelected;				//!<
	std::vector<Group3D*>				_sceneGroup;					//!< Model wrapper
	GPUDataBuffer						_sceneGPUData;					//!< Buffer of static GPU data objects, retrieved after building the BVH of each group

	// [FBO]
	std::vector<bool>					_computeShadowMap;				//!< Boolean value which determines if the shadow map must be computed again for a light
	GLuint								_nextFramebufferID;				//!< Framebuffer where we will render the next scene

	// [SSAO]
	SSAOFBO*							_ssaoFBO;						//!< FBO for ambient occlusion	
	Window*								_window;						//!< Source of queries for window size, etc

protected:
	/**
	*	@brief Renders the scene as a set of points.
	*	@param mModel Additional model matrix to be applied over the initial model matrix.
	*	@param rendParams Rendering parameters to be taken into account.
	*/
	virtual void drawAsPoints(const mat4& mModel, RenderingParameters* rendParams);

	/**
	*	@brief Renders the scene as a set of lines.
	*	@param mModel Additional model matrix to be applied over the initial model matrix.
	*	@param rendParams Rendering parameters to be taken into account.
	*/
	virtual void drawAsLines(const mat4& mModel, RenderingParameters* rendParams);

	/**
	*	@brief Renders the scene as a set of triangles.
	*	@param mModel Additional model matrix to be applied over the initial model matrix.
	*	@param rendParams Rendering parameters to be taken into account.
	*/
	virtual void drawAsTriangles(const mat4& mModel, RenderingParameters* rendParams);

	/**
	*	@brief Renders the scene as a set of triangles.
	*	@param mModel Additional model matrix to be applied over the initial model matrix.
	*	@param rendParams Rendering parameters to be taken into account.
	*/
	virtual void drawAsTriangles(Camera* camera, const mat4& mModel, RenderingParameters* rendParams);

	/**
	*	@brief Renders the scene so that every fragment take the color of its position.
	*	@param mModel Additional model matrix to be applied over the initial model matrix.
	*	@param rendParams Rendering parameters to be taken into account.
	*/
	virtual void drawAsTriangles4Position(const mat4& mModel, RenderingParameters* rendParams);

	/**
	*	@brief Renders the scene so that every fragment take the color of its normal.
	*	@param mModel Additional model matrix to be applied over the initial model matrix.
	*	@param rendParams Rendering parameters to be taken into account.
	*/
	virtual void drawAsTriangles4Normal(const mat4& mModel, RenderingParameters* rendParams);

	/**
	*	@brief Renders the scene as a set of triangles with no textures.
	*	@param mModel Additional model matrix to be applied over the initial model matrix.
	*	@param rendParams Rendering parameters to be taken into account.
	*/
	virtual void drawAsTriangles4Shadows(const mat4& mModel, RenderingParameters* rendParams);

	// -------------- Draw scene -------------

	/**
	*	@brief Decides which objects are going to be rendered as points.
	*	@param shader Rendering shader which is drawing the scene.
	*	@param shaderType Unique ID of "shader".
	*	@param matrix Vector of matrices, including view, projection, etc.
	*	@param rendParams Parameters which indicates how the scene is rendered.
	*/
	virtual void drawSceneAsPoints(RenderingShader* shader, RendEnum::RendShaderTypes shaderType, std::vector<mat4>* matrix, RenderingParameters* rendParams);

	/**
	*	@brief Decides which objects are going to be rendered as a wireframe mesh.
	*	@param shader Rendering shader which is drawing the scene.
	*	@param shaderType Unique ID of "shader".
	*	@param matrix Vector of matrices, including view, projection, etc.
	*	@param rendParams Parameters which indicates how the scene is rendered.
	*/
	virtual void drawSceneAsLines(RenderingShader* shader, RendEnum::RendShaderTypes shaderType, std::vector<mat4>* matrix, RenderingParameters* rendParams);

	/**
	*	@brief Decides which objects are going to be rendered as a triangle mesh.
	*	@param shader Rendering shader which is drawing the scene.
	*	@param shaderType Unique ID of "shader".
	*	@param matrix Vector of matrices, including view, projection, etc.
	*	@param rendParams Parameters which indicates how the scene is rendered.
	*/
	virtual void drawSceneAsTriangles(RenderingShader* shader, RendEnum::RendShaderTypes shaderType, std::vector<mat4>* matrix, RenderingParameters* rendParams);

	/**
	*	@brief Decides which objects are going to be rendered as a triangle mesh. Only the normal is calculated for each fragment.
	*	@param shader Rendering shader which is drawing the scene.
	*	@param shaderType Unique ID of "shader".
	*	@param matrix Vector of matrices, including view, projection, etc.
	*	@param rendParams Parameters which indicates how the scene is rendered.
	*/
	virtual void drawSceneAsTriangles4Normal(RenderingShader* shader, RendEnum::RendShaderTypes shaderType, std::vector<mat4>* matrix, RenderingParameters* rendParams);

	/**
	*	@brief Decides which objects are going to be rendered as a triangle mesh. Only the position is calculated for each fragment.
	*	@param shader Rendering shader which is drawing the scene.
	*	@param shaderType Unique ID of "shader".
	*	@param matrix Vector of matrices, including view, projection, etc.
	*	@param rendParams Parameters which indicates how the scene is rendered.
	*/
	virtual void drawSceneAsTriangles4Position(RenderingShader* shader, RendEnum::RendShaderTypes shaderType, std::vector<mat4>* matrix, RenderingParameters* rendParams);

	// --------------- SSAO -----------------
	
	/**
	*	@brief Binds the default framebuffer, i.e. 0.
	*/
	void bindDefaultFramebuffer(RenderingParameters* rendParams);

	/**
	*	@brief Draws both the SSAO texture and RGB scene together.
	*/
	void composeScene();

	/**
	*	@brief Builds the SSAO texture.
	*/
	void drawSSAOScene();

	// --------------- Load ----------------

	/**
	*	@brief To be implemented: define cameras to visualize the scene.
	*/
	virtual void loadCameras();

	/**
	*	@brief To be implemented: define lights of the scene.
	*/
	virtual void loadLights();

	/**
	*	@brief To be implemented: define models which take part of the scene.
	*/
	virtual void loadModels() = 0;

public:
	/**
	*	@brief Default constructor. Intializes some generic components.
	*/
	Scene();

	/**
	*	@brief Destructor.
	*/
	~Scene();

	/**
	*	@brief Computes collisions between any mesh active in the scene. 
	*/
	void computeCollisions();

	/**
	*	@brief Loads the elements from the scene: lights, cameras, models, etc.
	*/
	virtual void load();

	/**
	*	@brief  
	*/
	bool hit(Model3D::RayGPUData& ray);

	/**
	*	@brief Modifies the framebuffer where the next scene will be rendered.
	*	@param fboID OpenGL ID of new framebuffer.
	*/
	virtual void modifyNextFramebufferID(const GLuint fboID);

	/**
	*	@brief Resize event.
	*	@param width New canvas width.
	*	@param height New canvas height.
	*/
	virtual void modifySize(const uint16_t width, const uint16_t height);

	/**
	*	@brief Draws the scene as the rendering parameters specifies.
	*	@param mModel Additional model matrix to be applied over the initial model matrix.
	*	@param rendParams Rendering parameters to be taken into account.
	*/
	virtual void render(const mat4& mModel, RenderingParameters* rendParams);

	// ------------ Getters --------------

	/**
	*	@return Axis aligned bounding box which wraps the complete scene.
	*/
	AABB getAABB();

	/**
	*	@return Manager of application cameras.
	*/
	CameraManager* getCameraManager()  const { return _cameraManager.get(); }
	
	/**
	*	@return Pointer of vector with all registered model components.
	*/
	std::vector<Model3D::ModelComponent*> getModelComponents();

	/**
	*	@return Transformation matrix of the selected scene. 
	*/
	mat4 getGroupMatrix(GLint groupIndex) { return _sceneGroup[groupIndex]->getModelMatrix(); }
	
	/**
	*	@return Index of the selected model component.
	*/
	GLint getGroupSelected() const { return _modelCompSelected; }

	/**
	*	@brief Modifies the world transformation of a group. 
	*/
	void setGroupMatrix(GLint groupIndex, const mat4& modelMatrix) { _sceneGroup[groupIndex]->setModelMatrix(modelMatrix); }
};

