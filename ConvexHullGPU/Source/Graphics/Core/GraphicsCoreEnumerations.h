#pragma once

/**
*	@file GraphicsCoreEnumerations.h
*	@authors Alfonso López Ruiz (alr00048@red.ujaen.es)
*	@date 07/10/2019
*/

/**
*	@brief Contains any enum related to rendering process.
*/
struct RendEnum
{
	/// [Geometry and topology]

	// Topology types
	enum IBOTypes: uint8_t
	{
		IBO_POINT_CLOUD,
		IBO_WIREFRAME,
		IBO_TRIANGLE_MESH
	};

	// Geometry types
	enum VBOTypes : uint8_t
	{
		VBO_POSITION,
		VBO_NORMAL,
		VBO_TEXT_COORD,
		VBO_TANGENT,
		VBO_OFFSET,
		VBO_SCALE,
		VBO_ROTATION,
		VBO_COLOR_TEXT_COORD_X,
		VBO_RETURN_NUMBER,
		VBO_RETURN_DIVISION,
		VBO_INTENSITY
	};

	/**
	*	@return Number of VBO different types.
	*/
	const static GLsizei numIBOTypes() { return IBO_TRIANGLE_MESH + 1; }

	/**
	*	@return Number of VBO different types.
	*/
	const static GLsizei numVBOTypes() { return VBO_INTENSITY + 1; }

	/// [Shaders]

	// Shader implementations
	enum RendShaderTypes : uint8_t
	{
		// General
		WIREFRAME_SHADER,
		TRIANGLE_MESH_SHADER,
		TRIANGLE_MESH_NORMAL_SHADER,
		TRIANGLE_MESH_POSITION_SHADER,
		SHADOWS_SHADER,

		// Geometry
		GEOMETRY_CONE_SHADER,
		GEOMETRY_RANGE_SHADER,
		VERTEX_NORMAL_SHADER,

		// LiDAR Point cloud
		POINT_CLOUD_SHADER,
		POINT_CLOUD_HEIGHT_SHADER,

		// Data structures
		BVH_SHADER,

		// SSAO
		BLUR_SSAO_SHADER,
		SSAO_SHADER,

		// Debug quad
		DEBUG_QUAD_SHADER,

		// Filters
		BLUR_SHADER,
		NORMAL_MAP_SHADER
	};

	enum CompShaderTypes : uint8_t
	{
		// BVH
		BUILD_CLUSTER_BUFFER,
		BVH_COLLISION,
		CLUSTER_MERGING,
		COMPUTE_FACE_AABB,
		COMPUTE_GROUP_AABB,
		COMPUTE_MORTON_CODES,
		DOWN_SWEEP_PREFIX_SCAN,
		FIND_BEST_NEIGHBOR,
		MESH_BVH_COLLISION,
		REALLOCATE_CLUSTERS,
		REDUCE_PREFIX_SCAN,
		RESET_LAST_POSITION_PREFIX_SCAN,

		// Radix sort
		BIT_MASK_RADIX_SORT,
		END_LOOP_COMPUTATIONS,
		REALLOCATE_RADIX_SORT,
		RESET_BUFFER_INDEX,

		// Model
		COMPUTE_TANGENTS_1,
		COMPUTE_TANGENTS_2,
		MODEL_APPLY_MODEL_MATRIX,
		MODEL_MESH_GENERATION,
		PLANAR_SURFACE_GENERATION,
		PLANAR_SURFACE_TOPOLOGY,

		// CH
		COLLAPSE_POINT_BUFFER_CH,
		COMPUTE_LOWEST_ANGLE_SHADER,
		COMPUTE_MIN_Y,
		IDENTIFY_SIMILAR_POINTS_CH,
		RESET_8,
		RESET_64
	};

	/**
	*	@return Number of compute shaders.
	*/
	const static GLsizei numComputeShaderTypes() { return RESET_64 + 1; }

	/**
	*	@return Number of rendering shaders.
	*/
	const static GLsizei numRenderingShaderTypes() { return NORMAL_MAP_SHADER + 1; }

	/// [Rendering parameters]

	// Matrices types
	enum MatricesTypes : uint8_t
	{
		MODEL_MATRIX,
		VIEW_MATRIX,
		PROJ_MATRIX,
		VIEW_PROJ_MATRIX,
		BIAS_VIEW_PROJ_MATRIX
	};

	/**
	*	@return Number of rendering shaders.
	*/
	const static GLsizei numMatricesTypes() { return BIAS_VIEW_PROJ_MATRIX + 1; }
};