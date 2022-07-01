#pragma once

#include "Geometry/3D/Ray3D.h"
#include "Graphics/Core/Model3D.h"
#include "Graphics/Core/VAO.h"

class TriangleMesh;

/**
*	@file DrawRay3D.h
*	@authors Alfonso López Ruiz (alr00048@red.ujaen.es)
*	@date 08/16/2019
*/

/**
*	@brief Ray 3D painter.
*/
class DrawRay3D: public Model3D
{
protected:
	const static float EXCEED_BOUNDARIES;					//!< T for ray parametric equation

protected:
	Ray3D*	_ray;											//!< Ray to be queried abouts its geometry
	VAO*	_vao;											//!< Geometry and topology in GPU

protected:
	/**
	*	@brief Specifies geometry and topology in GPU.
	*/
	void createVAO();

public:
	/**
	*	@brief Constructor.
	*	@param ray Ray from where we get the ray points (orig and dest).
	*	@param modelMatrix First model transformation.
	*/
	DrawRay3D(Ray3D* ray, const mat4& modelMatrix = mat4(1.0f));

	/**
	*	@brief Destructor.
	*/
	~DrawRay3D();

	/**
	*	@brief Modifies model vertices from local space to world space.
	*/
	virtual void computeWorldPosition(mat4& modelMatrix) {}

	/**
	*	@brief Renders the ray as a set of lines.
	*	@param shader Shader which will draw the model.
	*	@param shaderType Index of shader at the list, so we can identify what type of uniform variables must be declared.
	*	@param matrix Vector of matrices which can be applied in the rendering process.
	*/
	virtual void drawAsLines(RenderingShader* shader, const RendEnum::RendShaderTypes shaderType, std::vector<mat4>& matrix);

	/**
	*	@brief Renders the ray as a set of points.
	*	@param shader Shader which will draw the model.
	*	@param shaderType Index of shader at the list, so we can identify what type of uniform variables must be declared.
	*	@param matrix Vector of matrices which can be applied in the rendering process.
	*/
	virtual void drawAsPoints(RenderingShader* shader, const RendEnum::RendShaderTypes shaderType, std::vector<mat4>& matrix);

	/**
	*	@brief Renders the ray as a set of triangles.
	*	@param shader Shader which will draw the model.
	*	@param shaderType Index of shader at the list, so we can identify what type of uniform variables must be declared.
	*	@param matrix Vector of matrices which can be applied in the rendering process.
	*/
	virtual void drawAsTriangles(RenderingShader* shader, const RendEnum::RendShaderTypes shaderType, std::vector<mat4>& matrix);

	/**
	*	@brief Renders the AABB as a set of triangles with no textures as we only want to color fragments taking into account its depth.
	*	@param shader Shader which will draw the model.
	*	@param shaderType Index of shader at the list, so we can identify what type of uniform variables must be declared.
	*	@param matrix Vector of matrices which can be applied in the rendering process.
	*/
	virtual void drawAsTriangles4Fog(RenderingShader* shader, const RendEnum::RendShaderTypes shaderType, std::vector<mat4>& matrix);

	/**
	*	@brief Renders the ray as a set of triangles with no textures (except disp. mapping) as we only want to retrieve the depth. 
	*	@param shader Shader which will draw the model.
	*	@param shaderType Index of shader at the list, so we can identify what type of uniform variables must be declared.
	*	@param matrix Vector of matrices which can be applied in the rendering process.
	*/
	virtual void drawAsTriangles4Shadows(RenderingShader* shader, const RendEnum::RendShaderTypes shaderType, std::vector<mat4>& matrix);

	/**
	*	@brief Renders the points captured by a sensor (coloured).
	*	@param shader Shader which will draw the model.
	*	@param shaderType Index of shader at the list, so we can identify what type of uniform variables must be declared.
	*	@param matrix Vector of matrices which can be applied in the rendering process.
	*/
	virtual void drawCapturedPoints(RenderingShader* shader, const RendEnum::RendShaderTypes shaderType, std::vector<mat4>& matrix);

	/**
	*	@brief Computes the ray data and sends it to GPU.
	*	@return Success of operation.
	*/
	virtual bool load(const mat4& modelMatrix = mat4(1.0f));
};

