#pragma once

#include "Graphics/Core/Model3D.h"
#include "objloader/OBJ_Loader.h"

/**
*	@file CADModel.h
*	@authors Alfonso López Ruiz (alr00048@red.ujaen.es)
*	@date 11/28/2020
*/

#define COMMENT_CHAR "#"

/**
*	@brief Model loaded from an OBJ file.
*/
class CADModel: public Model3D
{
protected:
	static std::unordered_map<std::string, std::unique_ptr<Material>> _cadMaterials;
	static std::unordered_map<std::string, std::unique_ptr<Texture>> _cadTextures;

public:
	const static std::string BINARY_EXTENSION;				//!< File extension for binary models
	const static std::string OBJ_EXTENSION;					//!< File extension for original OBJ models

protected:
	std::string		_filename;								//!< File path (without extension)
	std::string		_textureFolder;							//!< Folder where model textures may be located
	bool			_useBinary;								//!< Use binary file instead of original obj models

	// Shaders
	ComputeShader*	_bitMaskShader;
	ComputeShader*	_collapsePointBufferShader;
	ComputeShader*	_computeLowestAngleShader;
	ComputeShader*	_computeMinYShader;
	ComputeShader*	_downSweepShader;
	ComputeShader*	_eraseSimilarPointsShader;
	ComputeShader*	_reallocatePositionShader;
	ComputeShader*	_reduceShader;
	ComputeShader*	_reset8, * _reset64;
	ComputeShader*	_resetPositionShader;

	// GPU buffers
	GLuint			_int32SSBO, _int64SSBO;

protected:
	/**
	*	@brief Builds a new model component copying the geometry from the current one.
	*/
	ModelComponent* buildModelComp(ModelComponent* modelComp);

	/**
	*	@brief Builds a new model component copying the geometry from the current one.
	*/
	GLuint buildModelCompGPU(ModelComponent* modelComp, unsigned& numPoints);

	/**
	*	@return VAO with convex hull data.
	*/
	VAO* buildCHVAO(std::vector<Triangle3D>& triangles);

	/**
	*	@brief
	*/
	GLuint calculateMortonCodes(const GLuint pointsSSBO, unsigned numPoints, const AABB& aabb);

	/**
	*	@brief Computes a triangle mesh buffer composed only by indices.
	*/
	void computeMeshData(Model3D::ModelComponent* modelComp);

	/**
	*	@brief Creates a new material from the attributes of a model component.
	*/
	Material* createMaterial(ModelComponent* modelComp);

	/**
	*	@brief Initializes a model component with the content of a mesh.
	*/
	void createModelComponent(objl::Mesh* mesh, ModelComponent* modelComp);

	/**
	*	@brief Modifies those triangles that doesnt follow a counterclockwise order.
	*/
	static void flipTriangles(std::vector<Triangle3D>& triangles);

	/**
	*	@brief Generates geometry via GPU.
	*/
	void generateGeometryTopology(Model3D::ModelComponent* modelComp, const mat4& modelMatrix);

	/**
	*	@brief Searchs the class in keyMap of a model whose name is given by modelName.
	*/
	std::string getKeyValue(std::map<std::string, std::string>& keyMap, std::string& modelName, std::string& defaultClass);

	/**
	*	@brief Returns the index of the 2D point with lowest angle respect a coordinate system where startingPoint is the zero.
	*	@param startingPoint Index of the point that conforms the coordinate system and is excluded from the search.
	*/
	int getLowestAnglePoint(ModelComponent* modelComp, int startingPoint);

	/**
	*	@brief Returns the index of the 2D point with lowest angle respect a coordinate system where startingPoint is the zero.
	*	@param startingPoint Index of the point that conforms the coordinate system and is excluded from the search.
	*/
	int getLowestAnglePointGPU(const GLuint pointsSSBO, unsigned numPoints, int startingPoint);

	/**
	*	@return Index of point with minimum Y coordinate.
	*/
	int getMinY(ModelComponent* modelComp);

	/**
	*	@return Index of point with minimum Y coordinate.
	*/
	int getMinYGPU(GLuint pointBuffer, GLuint numPoints, const AABB& aabb);

	/**
	*	@brief Returns the index of a point which creates a triangle with b and c that leaves the rest of points at the same side.
	*	@param triangleC Triangle vertex excluded from the segment a and b which is the retrieved edge from the stack.
	*/
	int getNewPoint(ModelComponent* modelComp, int edgeA, int edgeB, int triangle);

	/**
	*	@brief Returns the index of a point which creates a triangle with b and c that leaves the rest of points at the same side.
	*	@param triangleC Triangle vertex excluded from the segment a and b which is the retrieved edge from the stack.
	*/
	int getNewPointGPU(ModelComponent* modelComp, int edgeA, int edgeB, int triangle);

	/**
	*	@brief Finds another vertice that creates a triangle with a and b and isolates the rest of points at certain side.
	*/
	int getTriangleThirdVertex(ModelComponent* modelComp, int a, int b);

	/**
	*	@brief Finds another vertice that creates a triangle with a and b and isolates the rest of points at certain side.
	*/
	int getTriangleThirdVertexGPU(int a, int b);

	/**
	*	@brief Checks if a point leaves the rest of the point cloud in a side.
	*/
	bool leavesPointsInASide(ModelComponent* modelComp, int currentPoint, int edgeA, int edgeB, int triangleC);

	/**
	*	@brief Fills the content of model component with binary file data.
	*/
	bool loadModelFromBinaryFile();

	/**
	*	@brief Generates geometry via GPU.
	*/
	bool loadModelFromOBJ(const mat4& modelMatrix);

	/**
	*	@brief Loads the CAD model from a binary file, if possible.
	*/
	bool readBinary(const std::string& filename, const std::vector<Model3D::ModelComponent*>& modelComp);

	/**
	*	@brief Loads all the pair class-value in a file.
	*/
	void readClassFile(const std::string& filename, std::map<std::string, std::string>& keyMap, std::string& defaultClass);

	/**
	*	@brief Communicates the model structure to GPU for rendering purposes.
	*/
	void setVAOData(ModelComponent* modelComp);

	/**
	*	@brief
	*/
	GLuint sortFacesByMortonCode(const GLuint mortonCodes, unsigned numPoints);

	/**
	*	@brief
	*/
	void sortPoints(GLuint pointBuffer, GLuint numPoints, const AABB& aabb);

	/**
	*	@brief Writes the model to a binary file in order to fasten the following executions.
	*	@return Success of writing process.
	*/
	bool writeToBinary();

public:
	/**
	*	@brief CADModel constructor.
	*	@param filename Path where the model is located.
	*	@param useBinary Use of previously written binary files.
	*/
	CADModel(const std::string& filename, const std::string& textureFolder, const bool useBinary);

	/**
	*	@brief Deleted copy constructor.
	*	@param model Model to copy attributes.
	*/
	CADModel(const CADModel& model) = delete;

	/**
	*	@brief Destructor.
	*/
	virtual ~CADModel();

	/**
	*	@brief Computes the convex hull of the mesh points. 
	*/
	VAO* computeConvexHullCPU(int& numIndices);

	/**
	*	@brief Computes the convex hull of the mesh points.
	*/
	VAO* computeConvexHullGPU(const AABB& aabb, int& numIndices);

	/**
	*	@brief Loads the model data from file.
	*	@return Success of operation.
	*/
	virtual bool load(const mat4& modelMatrix = mat4(1.0f));

	/**
	*	@brief Deleted assignment operator.
	*	@param model Model to copy attributes.
	*/
	CADModel& operator=(const CADModel& model) = delete;

protected:
	/**
	*	@brief Maintains a tuple of integer values that represent two indexes of an array.
	*/
	class IndexTuple
	{
	public:
		int _orig, _dest;							// Indexes of the segment points
		int _triangleC;								// Index of the third vertex of the triangle
		int _hash;									// Identifier of the face

	public:
		/**
		*	@brief Constructor.
		*/
		IndexTuple(int orig, int dest, int triangleC);

		/**
		*	@brief Destructor.
		*/
		~IndexTuple();

		/**
		*	@brief Devuelve un entero identificador del eje.
		*/
		int getEdgeHash();

		/**
		*	@brief Assignment operator overriding.
		*/
		IndexTuple& operator=(const IndexTuple& tuple);

		/**
		*	@brief Equal operator overriding.
		*/
		bool operator==(const IndexTuple& tuple) const;

		/**
		*	@brief Equal operator overriding.
		*/
		bool operator!=(const IndexTuple& tuple) const;
	};

	struct Face
	{
		uvec3				_indices;
		ModelComponent*		_modelComp;

		/**
		*	@brief Alters the order the indices whether they are not CCW-defined.
		*/
		void checkCCW();

		/**
		*	@return True if two faces are equal.
		*/
		bool operator==(const Face& face) const;
	};

	/**
	*	@brief Comparison struct to save index tuples on a set.
	*/
	struct FaceEquals {
	public:
		bool operator()(const Face& a, const Face& b) const;
	};

	/**
	*	@brief Hash value for any index tuple (unique).
	*/
	struct FaceHash {
	public:
		size_t operator()(const Face& tuple) const;
	};

	/**
	*	@brief Comparison struct to save index tuples on a set.
	*/
	struct IndexTupleEquals {
	public:
		bool operator()(const IndexTuple& a, const IndexTuple& b) const;
	};

	/**
	*	@brief Hash value for any index tuple (unique).
	*/
	struct IndexTupleHash {
	public:
		size_t operator()(const IndexTuple& tuple) const;
	};

public:
	/**
	*	@brief Maintains the data of the process that extract the convex hull of a set of points.
	*/
	class ConvexHullProcessData
	{
	public:
		std::unordered_set<IndexTuple, IndexTupleHash, IndexTupleEquals>	_boundaryCH;				// Indices of segments points to be checked
		std::unordered_set<Face, FaceHash, FaceEquals>						_includedFace;				// Faces which are already included in the mesh
		std::unordered_set<int>												_pointIndex;				// Already included vertices
		std::vector<Triangle3D>												_triangles;					// Already formed triangles

		// Aditional information for step-by-step process
		std::vector<vec3>													_filteredPoints;
		ModelComponent*														_modelComp;

	public:
		/**
		*	@brief Constructor. Initializes the data structure with the first three points.
		*/
		ConvexHullProcessData(ModelComponent* modelComp, int segmentA, int segmentB, int triangleC);

		/**
		*	@brief Updates the process structure by trying to include a new segment (segmentA, segmentB) and a third point of a triangle (triangleC).
		*/
		void updateData(int segmentA, int segmentB, int triangleC);
	};
};

