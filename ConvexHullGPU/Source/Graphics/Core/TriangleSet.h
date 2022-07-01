#pragma once

/**
*	@file Group3D.h
*	@authors Alfonso López Ruiz (alr00048@red.ujaen.es)
*	@date 07/09/2019
*/

#include "Graphics/Core/Model3D.h"

/**
*	@brief Wrapper of a set of triangles.
*/
class TriangleSet: public Model3D
{
public:
	/**
	*	@brief Default constructor. 
	*/
	TriangleSet();

	/**
	*	@brief Destructor. 
	*/
	~TriangleSet();

	/**
	*	@brief Erases all vertices and faces from the current set. 
	*/
	void clear();

	/**
	*	@brief  
	*/
	void insertFaces(std::vector<VertexGPUData>* vertices, FaceGPUData* faces, GLuint numFaces);

	/**
	*	@brief Computes the model data and sends it to GPU.
	*	@param modelMatrix Model matrix to be applied while generating geometry.
	*	@return Success of operation.
	*/
	virtual bool load(const mat4& modelMatrix = mat4(1.0f));
};

