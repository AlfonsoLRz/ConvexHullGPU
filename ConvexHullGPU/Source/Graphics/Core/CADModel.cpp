#include "stdafx.h"
#include "CADModel.h"

#include <filesystem>
#include "Geometry/3D/Triangle3D.h"
#include "Graphics/Application/MaterialList.h"
#include "Graphics/Core/ComputeShader.h"
#include "Graphics/Core/ShaderList.h"
#include "Graphics/Core/VAO.h"
#include "Utilities/FileManagement.h"
#include "Utilities/ChronoUtilities.h"

// Initialization of static attributes
std::unordered_map<std::string, std::unique_ptr<Material>> CADModel::_cadMaterials;
std::unordered_map<std::string, std::unique_ptr<Texture>> CADModel::_cadTextures;

const std::string CADModel::BINARY_EXTENSION = ".bin";
const std::string CADModel::OBJ_EXTENSION = ".obj";

/// [Public methods]

CADModel::CADModel(const std::string& filename, const std::string& textureFolder, const bool useBinary) : 
	Model3D(mat4(1.0f), 1)
{
	_filename = filename;
	_textureFolder = textureFolder;
	_useBinary = useBinary;

	// Shaders
	ShaderList* shaderList = ShaderList::getInstance();

	_bitMaskShader = shaderList->getComputeShader(RendEnum::BIT_MASK_RADIX_SORT);
	_collapsePointBufferShader = shaderList->getComputeShader(RendEnum::COLLAPSE_POINT_BUFFER_CH);
	_computeLowestAngleShader = shaderList->getComputeShader(RendEnum::COMPUTE_LOWEST_ANGLE_SHADER);
	_computeMinYShader = shaderList->getComputeShader(RendEnum::COMPUTE_MIN_Y);
	_downSweepShader = shaderList->getComputeShader(RendEnum::DOWN_SWEEP_PREFIX_SCAN);
	_eraseSimilarPointsShader = shaderList->getComputeShader(RendEnum::IDENTIFY_SIMILAR_POINTS_CH);
	_reduceShader = shaderList->getComputeShader(RendEnum::REDUCE_PREFIX_SCAN);
	_resetPositionShader = shaderList->getComputeShader(RendEnum::RESET_LAST_POSITION_PREFIX_SCAN);
	_reset8 = shaderList->getComputeShader(RendEnum::RESET_8);
	_reset64 = shaderList->getComputeShader(RendEnum::RESET_64);
	_reallocatePositionShader = ShaderList::getInstance()->getComputeShader(RendEnum::REALLOCATE_RADIX_SORT);

	_int32SSBO = ComputeShader::setWriteBuffer(unsigned(), 1, GL_DYNAMIC_DRAW);
	_int64SSBO = ComputeShader::setWriteBuffer(uint64_t(), 1, GL_DYNAMIC_DRAW);
}

CADModel::~CADModel()
{
	glDeleteBuffers(1, &_int32SSBO);
	glDeleteBuffers(1, &_int64SSBO);
}

VAO* CADModel::computeConvexHullCPU(int& numIndices)
{
	for (ModelComponent* modelComp : _modelComp)
	{
		ModelComponent* nModelComp = this->buildModelComp(modelComp);
		int min_y = this->getMinY(nModelComp);
		int lowestAlphaIndex = getLowestAnglePoint(nModelComp, min_y);
		int triangleC = getTriangleThirdVertex(nModelComp, min_y, lowestAlphaIndex);

		ConvexHullProcessData data(nModelComp, min_y, lowestAlphaIndex, triangleC);
		int j = 0;

		while (!data._boundaryCH.empty())
		{
			// Retrieve a new segment and delete it
			auto it = data._boundaryCH.begin();
			IndexTuple segment = *it;
			data._boundaryCH.erase(it);

			int newPointIndex = getNewPoint(nModelComp, segment._orig, segment._dest, segment._triangleC);
			if (newPointIndex == -1) continue;				// Some points are equal to others (even if indexes are distinct). In those cases no point is found

			data.updateData(segment._orig, segment._dest, newPointIndex);
			++j;
		}

		data._triangles;
		this->flipTriangles(data._triangles);

		numIndices = data._triangles.size() * 4;
		delete nModelComp;

		return this->buildCHVAO(data._triangles);
	}

	return nullptr;
}

VAO* CADModel::computeConvexHullGPU(const AABB& aabb, int& numIndices)
{
	for (ModelComponent* modelComp : _modelComp)
	{
		unsigned numPoints;

		// Preprocessing the model comp to conver it to point cloud without repeating points
		GLuint pointBufferSSBO = this->buildModelCompGPU(modelComp, numPoints);
		this->sortPoints(pointBufferSSBO, numPoints, aabb);

		GLuint min_y = this->getMinYGPU(pointBufferSSBO, numPoints, aabb);
		GLuint lowestAlphaIndex = this->getLowestAnglePointGPU(pointBufferSSBO, numPoints, min_y);
		GLuint triangleC = getTriangleThirdVertexGPU(min_y, lowestAlphaIndex);

		//ConvexHullProcessData data(nModelComp, min_y, lowestAlphaIndex, triangleC);
		//int j = 0;

		//while (!data._boundaryCH.empty())
		//{
		//	// Retrieve a new segment and delete it
		//	auto it = data._boundaryCH.begin();
		//	IndexTuple segment = *it;
		//	data._boundaryCH.erase(it);

		//	int newPointIndex = getNewPoint(nModelComp, segment._orig, segment._dest, segment._triangleC);
		//	if (newPointIndex == -1) continue;				// Some points are equal to others (even if indexes are distinct). In those cases no point is found

		//	data.updateData(segment._orig, segment._dest, newPointIndex);
		//	++j;
		//}

		//data._triangles;
		//this->flipTriangles(data._triangles);

		//numIndices = data._triangles.size() * 4;
		//delete nModelComp;

		//return this->buildCHVAO(data._triangles);
	}

	return nullptr;
}

bool CADModel::load(const mat4& modelMatrix)
{
	if (!_loaded)
	{
		bool success = false, binaryExists = false;

		if (_useBinary && (binaryExists = std::filesystem::exists(_filename + BINARY_EXTENSION)))
		{
			success = this->loadModelFromBinaryFile();
		}
			
		if (!success)
		{
			success = this->loadModelFromOBJ(modelMatrix);
		}

		if (!binaryExists && success)
		{
			this->writeToBinary();
		}

		return (_loaded = true);
	}

	return false;
}

/// [Protected methods]

Model3D::ModelComponent* CADModel::buildModelComp(ModelComponent* modelComp)
{
	ModelComponent* nModelComp = new ModelComponent(nullptr);
	nModelComp->_geometry = modelComp->_geometry;

	for (int i = 0; i < nModelComp->_geometry.size(); ++i)
	{
		for (int j = i + 1; j < nModelComp->_geometry.size(); ++j)
		{
			if (glm::distance(nModelComp->_geometry[i]._position, nModelComp->_geometry[j]._position) < glm::epsilon<float>())
			{
				nModelComp->_geometry.erase(nModelComp->_geometry.begin() + j);
				--j;
			}
		}
	}

	return nModelComp;
}

GLuint CADModel::buildModelCompGPU(ModelComponent* modelComp, unsigned& numPoints)
{
	int zero = 0;
	GLuint numGroups = ComputeShader::getNumGroups(modelComp->_geometry.size());
	GLuint geometrySSBO = ComputeShader::setReadBuffer(modelComp->_geometry, GL_DYNAMIC_DRAW);
	GLuint geometry2SSBO = ComputeShader::setWriteBuffer(VertexGPUData(), modelComp->_geometry.size(), GL_DYNAMIC_DRAW);
	GLuint invalidSSBO = ComputeShader::setWriteBuffer(uint8_t(), modelComp->_geometry.size(), GL_DYNAMIC_DRAW);

	ComputeShader::updateReadBuffer(_int32SSBO, &zero, 1, GL_DYNAMIC_DRAW);

	_reset8->bindBuffers(std::vector<GLuint> { invalidSSBO });
	_reset8->use();
	_reset8->setUniform("arraySize", GLuint(modelComp->_geometry.size()));
	_reset8->setUniform("value", GLuint(0));
	_reset8->execute(numGroups, 1, 1, ComputeShader::getMaxGroupSize(), 1, 1);

	_eraseSimilarPointsShader->bindBuffers(std::vector<GLuint> { geometrySSBO, invalidSSBO });
	_eraseSimilarPointsShader->use();
	_eraseSimilarPointsShader->setUniform("arraySize", GLuint(modelComp->_geometry.size()));
	_eraseSimilarPointsShader->setUniform("radius", GLuint(100));
	_eraseSimilarPointsShader->execute(numGroups, 1, 1, ComputeShader::getMaxGroupSize(), 1, 1);

	uint8_t* validity = ComputeShader::readData(invalidSSBO, uint8_t());
	std::vector<uint8_t> validBuffer = std::vector<uint8_t>(validity, validity + modelComp->_geometry.size());

	_collapsePointBufferShader->bindBuffers(std::vector<GLuint> { geometrySSBO, invalidSSBO, geometry2SSBO, _int32SSBO });
	_collapsePointBufferShader->use();
	_collapsePointBufferShader->setUniform("arraySize", GLuint(modelComp->_geometry.size()));
	_collapsePointBufferShader->execute(numGroups, 1, 1, ComputeShader::getMaxGroupSize(), 1, 1);

	glDeleteBuffers(1, &geometrySSBO);
	glDeleteBuffers(1, &invalidSSBO);

	numPoints = *ComputeShader::readData(_int32SSBO, GLuint());

	return geometry2SSBO;
}

VAO* CADModel::buildCHVAO(std::vector<Triangle3D>& triangles)
{
	VAO* chVAO = new VAO(true);
	std::vector<VertexGPUData> vertices;
	std::vector<GLuint> indices;
	unsigned index;

	for (Triangle3D& triangle : triangles)
	{
		index = vertices.size();
		vec3 normal = triangle.normal();

		vertices.push_back(VertexGPUData{ triangle.getP1(), .0f, normal });
		vertices.push_back(VertexGPUData{ triangle.getP2(), .0f, normal });
		vertices.push_back(VertexGPUData{ triangle.getP3(), .0f, normal });
		indices.insert(indices.end(), { index + 0, index + 1, index + 2, Model3D::RESTART_PRIMITIVE_INDEX });
	}

	chVAO->setVBOData(vertices);
	chVAO->setIBOData(RendEnum::IBO_TRIANGLE_MESH, indices);

	return chVAO;
}

GLuint CADModel::calculateMortonCodes(const GLuint pointsSSBO, unsigned numPoints, const AABB& aabb)
{
	ComputeShader* computeMortonShader = ShaderList::getInstance()->getComputeShader(RendEnum::COMPUTE_MORTON_CODES);

	const int numGroups = ComputeShader::getNumGroups(numPoints);
	const GLuint mortonCodeBuffer = ComputeShader::setWriteBuffer(unsigned(), numPoints);

	computeMortonShader->bindBuffers(std::vector<GLuint> { pointsSSBO, mortonCodeBuffer });
	computeMortonShader->use();
	computeMortonShader->setUniform("arraySize", numPoints);
	computeMortonShader->setUniform("sceneMaxBoundary", aabb.max());
	computeMortonShader->setUniform("sceneMinBoundary", aabb.min());
	computeMortonShader->execute(numGroups, 1, 1, ComputeShader::getMaxGroupSize(), 1, 1);

	return mortonCodeBuffer;
}

void CADModel::computeMeshData(ModelComponent* modelComp)
{
	ComputeShader* shader = ShaderList::getInstance()->getComputeShader(RendEnum::MODEL_MESH_GENERATION);
	const int arraySize = modelComp->_topology.size();
	const int numGroups = ComputeShader::getNumGroups(arraySize);

	GLuint modelBufferID, meshBufferID, outBufferID;
	modelBufferID = ComputeShader::setReadBuffer(modelComp->_geometry);
	meshBufferID = ComputeShader::setReadBuffer(modelComp->_topology);
	outBufferID = ComputeShader::setWriteBuffer(GLuint(), arraySize * 4);

	shader->bindBuffers(std::vector<GLuint> { modelBufferID, meshBufferID, outBufferID });
	shader->use();
	shader->setUniform("size", arraySize);
	shader->setUniform("restartPrimitiveIndex", Model3D::RESTART_PRIMITIVE_INDEX);
	shader->execute(numGroups, 1, 1, ComputeShader::getMaxGroupSize(), 1, 1);

	FaceGPUData* faceData = shader->readData(meshBufferID, FaceGPUData());
	GLuint* rawMeshData = shader->readData(outBufferID, GLuint());
	modelComp->_topology = std::move(std::vector<FaceGPUData>(faceData, faceData + arraySize));
	modelComp->_triangleMesh = std::move(std::vector<GLuint>(rawMeshData, rawMeshData + arraySize * 4));

	glDeleteBuffers(1, &modelBufferID);
	glDeleteBuffers(1, &meshBufferID);
	glDeleteBuffers(1, &outBufferID);
}

Material* CADModel::createMaterial(ModelComponent* modelComp)
{
	static const std::string nullMaterialName = "None";

	Material* material = MaterialList::getInstance()->getMaterial(CGAppEnum::MATERIAL_CAD_WHITE);
	Model3D::ModelComponentDescription* modelDescription = &modelComp->_modelDescription;
	std::string name = std::string(modelDescription->_materialName);
	std::string mapKd = std::string(modelDescription->_mapKd);
	std::string mapKs = std::string(modelDescription->_mapKs);

	if (!name.empty() && name != nullMaterialName)
	{
		auto itMaterial = _cadMaterials.find(name);

		if (itMaterial == _cadMaterials.end())
		{
			material = new Material();
			Texture* kad, * ks;

			if (!mapKd.empty())
			{
				kad = new Texture(_textureFolder + mapKd);
			}
			else
			{
				kad = new Texture(vec4(modelDescription->_kd, 1.0f));
			}

			if (!mapKs.empty())
			{
				ks = new Texture(_textureFolder + mapKs);
			}
			else
			{
				ks = new Texture(vec4(modelDescription->_ks, 1.0f));
				material->setShininess(modelDescription->_ns);
			}

			material->setTexture(Texture::KAD_TEXTURE, kad);
			material->setTexture(Texture::KS_TEXTURE, ks);

			_cadMaterials[name] = std::unique_ptr<Material>(material);
			_cadTextures[name + "-kad"] = std::unique_ptr<Texture>(kad);
			_cadTextures[name + "-ks"] = std::unique_ptr<Texture>(ks);
		}
		else
		{
			material = itMaterial->second.get();
		}
	}

	return material;
}

void CADModel::createModelComponent(objl::Mesh* mesh, ModelComponent* modelComp)
{
	VertexGPUData vertexData;

	for (int j = 0; j < mesh->Vertices.size(); j++)
	{
		vertexData._position = vec3(mesh->Vertices[j].Position.X, mesh->Vertices[j].Position.Y, mesh->Vertices[j].Position.Z);
		vertexData._normal = vec3(mesh->Vertices[j].Normal.X, mesh->Vertices[j].Normal.Y, mesh->Vertices[j].Normal.Z);
		vertexData._textCoord = vec2(mesh->Vertices[j].TextureCoordinate.X, mesh->Vertices[j].TextureCoordinate.Y);

		modelComp->_geometry.push_back(vertexData);
	}

	for (int j = 0; j < mesh->Indices.size(); j += 3)
	{
		modelComp->_topology.push_back(Model3D::FaceGPUData());
		modelComp->_topology[j / 3]._vertices = uvec3(mesh->Indices[j], mesh->Indices[j + 1], mesh->Indices[j + 2]);
		modelComp->_topology[j / 3]._modelCompID = modelComp->_id;
	}

	if (!modelComp->_material)
	{
		modelComp->_modelDescription = Model3D::ModelComponentDescription(mesh);
		modelComp->_material = this->createMaterial(modelComp);
		modelComp->setName(modelComp->_modelDescription._modelName);
	}
}

void CADModel::flipTriangles(std::vector<Triangle3D>& triangles)
{
	for (int triangle = 0; triangle < triangles.size(); ++triangle)
	{
		if (triangles[triangle].area() < .0f)
		{
			triangles[triangle] = Triangle3D(triangles[triangle].getP1(), triangles[triangle].getP2(), triangles[triangle].getP3());
		}
	}
}

void CADModel::generateGeometryTopology(Model3D::ModelComponent* modelComp, const mat4& modelMatrix)
{
	ComputeShader* shader = ShaderList::getInstance()->getComputeShader(RendEnum::MODEL_APPLY_MODEL_MATRIX);
	const int arraySize = modelComp->_geometry.size();
	const int numGroups = ComputeShader::getNumGroups(arraySize);

	GLuint modelBufferID;
	modelBufferID = ComputeShader::setReadBuffer(modelComp->_geometry);

	shader->bindBuffers(std::vector<GLuint> { modelBufferID});
	shader->use();
	shader->setUniform("mModel", modelMatrix * _modelMatrix);
	shader->setUniform("size", arraySize);
	if (modelComp->_material) modelComp->_material->applyMaterial4ComputeShader(shader);
	shader->execute(numGroups, 1, 1, ComputeShader::getMaxGroupSize(), 1, 1);

	VertexGPUData* data = shader->readData(modelBufferID, VertexGPUData());
	modelComp->_geometry = std::move(std::vector<VertexGPUData>(data, data + arraySize));

	this->computeTangents(modelComp);
	this->computeMeshData(modelComp);

	glDeleteBuffers(1, &modelBufferID);

	// Wireframe & point cloud are derived from previous operations
	modelComp->buildPointCloudTopology();
	modelComp->buildWireframeTopology();
}

std::string CADModel::getKeyValue(std::map<std::string, std::string>& keyMap, std::string& modelName, std::string& defaultClass)
{
	auto itFirst = keyMap.begin();
	unsigned maxMatches = 0, lastMatches;
	std::string currentClass = defaultClass;

	while (itFirst != keyMap.end())
	{
		auto itString = std::string(modelName).find(itFirst->first);

		if (itString != std::string::npos)
		{
			lastMatches = itFirst->first.size();

			if (lastMatches >= maxMatches)
			{
				maxMatches = lastMatches;
				currentClass = itFirst->second;
			}
		}

		++itFirst;
	}

	return currentClass;
}

int CADModel::getLowestAnglePoint(ModelComponent* modelComp, int startingPoint)
{
	// We ll find the point (2D) with lowest angle respect to the X axis of a coordinate system where startingPoint is the zero.
	vec3 zero = modelComp->_geometry[startingPoint]._position;
	float lowestAngle = INT_MAX;
	int lowestAngleIndex = startingPoint;

	#pragma omp parallel for
	for (int ind = 0; ind < modelComp->_geometry.size(); ++ind)
	{
		if (ind != startingPoint && glm::distance(zero, modelComp->_geometry[ind]._position) > glm::epsilon<float>())
		{
			vec3 point = modelComp->_geometry[ind]._position - zero;
			float alpha = std::atan2(point.y, point.x);

			#pragma omp critical
			{
				if (alpha < lowestAngle)
				{
					lowestAngle = alpha;
					lowestAngleIndex = ind;
				}
			}
		}
	}

	return lowestAngleIndex;
}

int CADModel::getLowestAnglePointGPU(const GLuint pointsSSBO, unsigned numPoints, int startingPoint)
{
	const int numGroups = ComputeShader::getNumGroups(numPoints);
	const uint64_t max64 = UINT64_MAX;
	ComputeShader::updateReadBuffer(_int64SSBO, &max64, 1, GL_DYNAMIC_DRAW);

	_computeLowestAngleShader->bindBuffers(std::vector<GLuint> { pointsSSBO, _int64SSBO });
	_computeLowestAngleShader->use();
	_computeLowestAngleShader->setUniform("arraySize", numPoints);
	_computeLowestAngleShader->setUniform("startingPoint", unsigned(startingPoint));
	_computeLowestAngleShader->execute(numGroups, 1, 1, ComputeShader::getMaxGroupSize(), 1, 1);

	return (*ComputeShader::readData(_int64SSBO, int64_t())) & 0x00000000ffffffff;
}

int CADModel::getMinY(ModelComponent* modelComp)
{
	float min_y = FLT_MAX;
	int min_y_index = 0;

	for (int i = 0; i < modelComp->_geometry.size(); ++i)
	{
		if (modelComp->_geometry[i]._position.y < min_y)
		{
			min_y = modelComp->_geometry[i]._position.y;
			min_y_index = i;
		}
	}

	return min_y_index;
}

int CADModel::getMinYGPU(GLuint pointBuffer, GLuint numPoints, const AABB& aabb)
{
	const int numGroups = ComputeShader::getNumGroups(numPoints);
	const uint64_t max64 = UINT64_MAX;
	ComputeShader::updateReadBuffer(_int64SSBO, &max64, 1, GL_DYNAMIC_DRAW);

	_computeMinYShader->bindBuffers(std::vector<GLuint> { pointBuffer, _int64SSBO });
	_computeMinYShader->use();
	_computeMinYShader->setUniform("aabbMin", aabb.min());
	_computeMinYShader->setUniform("arraySize", numPoints);
	_computeMinYShader->execute(numGroups, 1, 1, ComputeShader::getMaxGroupSize(), 1, 1);

	return (*ComputeShader::readData(_int64SSBO, int64_t())) & 0x00000000ffffffff;
}

int CADModel::getNewPoint(ModelComponent* modelComp, int edgeA, int edgeB, int triangleC)
{
	Triangle3D triangle(modelComp->_geometry[edgeA]._position, modelComp->_geometry[edgeB]._position, modelComp->_geometry[triangleC]._position);
	int lowestAlphaPoint = -1, greatestAlphaPoint = -1;
	float lowestAlpha = INT_MAX, greatestAlpha = INT_MIN;

	for (int i = 0; i < modelComp->_geometry.size(); ++i)
	{
		if (i != edgeA && i != edgeB && i != triangleC)
		{
			Triangle3D currentTriangle(modelComp->_geometry[edgeA]._position, modelComp->_geometry[edgeB]._position, modelComp->_geometry[i]._position);
			float alpha = triangle.getAlpha(currentTriangle);

			if (alpha < lowestAlpha)
			{
				lowestAlpha = alpha;
				lowestAlphaPoint = i;
			}

			if (alpha > greatestAlpha)
			{
				greatestAlpha = alpha;
				greatestAlphaPoint = i;
			}
		}
	}

	if (leavesPointsInASide(modelComp, lowestAlphaPoint, edgeA, edgeB, triangleC))
	{
		return lowestAlphaPoint;
	}

	if (leavesPointsInASide(modelComp, greatestAlphaPoint, edgeA, edgeB, triangleC))
	{
		return greatestAlphaPoint;
	}

	return -1;
}

int CADModel::getTriangleThirdVertex(ModelComponent* modelComp, int a, int b)
{
	bool validPoint = false;
	int i = 0;

	while (i < modelComp->_geometry.size() && !validPoint)
	{
		if (i != a && i != b)
		{
			Triangle3D triangle(modelComp->_geometry[a]._position, modelComp->_geometry[b]._position, modelComp->_geometry[i]._position);
			int relativePosition = -1, j = 0;
			validPoint = true;

			while (j < modelComp->_geometry.size() && relativePosition == -1)
			{
				if (j != a && j != b && j != i)
				{
					relativePosition = triangle.classify(modelComp->_geometry[j]._position);
				}

				++j;
			}

			while (j < modelComp->_geometry.size() && validPoint)
			{
				if (j != a && j != b && j != i)
				{
					int newPos = triangle.classify(modelComp->_geometry[j]._position);

					if (newPos != relativePosition)
					{
						validPoint = false;
					}
				}

				++j;
			}
		}

		++i;
	}

	return i - 1;
}

int CADModel::getTriangleThirdVertexGPU(int a, int b)
{
	bool validPoint = false;
	int i = 0;

	/*while (i < modelComp->_geometry.size() && !validPoint)
	{
		if (i != a && i != b)
		{
			Triangle3D triangle(modelComp->_geometry[a]._position, modelComp->_geometry[b]._position, modelComp->_geometry[i]._position);
			int relativePosition = -1, j = 0;
			validPoint = true;

			while (j < modelComp->_geometry.size() && relativePosition == -1)
			{
				if (j != a && j != b && j != i)
				{
					relativePosition = triangle.classify(modelComp->_geometry[j]._position);
				}

				++j;
			}

			while (j < modelComp->_geometry.size() && validPoint)
			{
				if (j != a && j != b && j != i)
				{
					int newPos = triangle.classify(modelComp->_geometry[j]._position);

					if (newPos != relativePosition)
					{
						validPoint = false;
					}
				}

				++j;
			}
		}

		++i;
	}*/

	return i - 1;
}

bool CADModel::leavesPointsInASide(ModelComponent* modelComp, int currentPoint, int edgeA, int edgeB, int triangleC)
{
	Triangle3D triangle(modelComp->_geometry[edgeA]._position, modelComp->_geometry[edgeB]._position, modelComp->_geometry[currentPoint]._position);
	int relativePosition = triangle.classify(modelComp->_geometry[triangleC]._position), j = 0;				// As triangleC point need to be necessarily a point cloud point we can take as the reference point to compare positions
	bool validPoint = true;

	while (j < modelComp->_geometry.size() && validPoint)
	{
		if (j != edgeA && j != edgeB && j != currentPoint)
		{
			int newPos = triangle.classify(modelComp->_geometry[j]._position);

			if (newPos != relativePosition && newPos != Triangle3D::COPLANAR)				// Comment the collinear check for the slower method of getting the new point
			{
				validPoint = false;
			}
		}

		++j;
	}

	return validPoint;
}

bool CADModel::loadModelFromBinaryFile()
{
	bool success;

	if (success = this->readBinary(_filename +  BINARY_EXTENSION, _modelComp))
	{
		for (ModelComponent* modelComp : _modelComp)
		{
			modelComp->_material = this->createMaterial(modelComp);
			modelComp->setName(modelComp->_modelDescription._modelName);
			this->setVAOData(modelComp);
		}
	}

	return success;
}

bool CADModel::loadModelFromOBJ(const mat4& modelMatrix)
{
	objl::Loader loader;
	bool success = loader.LoadFile(_filename + OBJ_EXTENSION);
	unsigned int numNameCollisions = 0;

	if (success)
	{
		std::string modelName, currentModelName;

		if (loader.LoadedMeshes.size())
		{
			unsigned index = 0;

			while (index < loader.LoadedMeshes.size() && modelName.empty())
			{
				modelName = loader.LoadedMeshes[0].MeshName;
				++index;
			}
		}

		while (_modelComp.size() < loader.LoadedMeshes.size())
		{
			_modelComp.push_back(new ModelComponent(this));
		}

		for (int i = 0; i < loader.LoadedMeshes.size(); i++)
		{
			this->createModelComponent(&(loader.LoadedMeshes[i]), _modelComp[i]);
			this->generateGeometryTopology(_modelComp[i], modelMatrix);
			this->setVAOData(_modelComp[i]);

			currentModelName = std::string(_modelComp[i]->_modelDescription._modelName);
			numNameCollisions += currentModelName == modelName;
		}

		if (_modelComp.size() > 1 && numNameCollisions > std::ceil(loader.LoadedMeshes.size() / 5))
		{
			for (ModelComponent* modelComp : _modelComp)
			{
				strcpy(modelComp->_modelDescription._modelName, modelComp->_modelDescription._materialName);
			}
		}
	}

	return success;
}

bool CADModel::readBinary(const std::string& filename, const std::vector<Model3D::ModelComponent*>& modelComp)
{
	std::ifstream fin(filename, std::ios::in | std::ios::binary);
	if (!fin.is_open())
	{
		return false;
	}

	size_t numModelComps, numVertices, numTriangles, numIndices;

	fin.read((char*)&numModelComps, sizeof(size_t));
	while (_modelComp.size() < numModelComps)
	{
		_modelComp.push_back(new ModelComponent(this));
	}

	for (Model3D::ModelComponent* model : modelComp)
	{
		fin.read((char*)&numVertices, sizeof(size_t));
		model->_geometry.resize(numVertices);
		fin.read((char*)&model->_geometry[0], numVertices * sizeof(Model3D::VertexGPUData));

		fin.read((char*)&numTriangles, sizeof(size_t));
		model->_topology.resize(numTriangles);
		fin.read((char*)&model->_topology[0], numTriangles * sizeof(Model3D::FaceGPUData));

		fin.read((char*)&numIndices, sizeof(size_t));
		model->_triangleMesh.resize(numIndices);
		fin.read((char*)&model->_triangleMesh[0], numIndices * sizeof(GLuint));

		fin.read((char*)&numIndices, sizeof(size_t));
		model->_pointCloud.resize(numIndices);
		fin.read((char*)&model->_pointCloud[0], numIndices * sizeof(GLuint));

		fin.read((char*)&numIndices, sizeof(size_t));
		model->_wireframe.resize(numIndices);
		fin.read((char*)&model->_wireframe[0], numIndices * sizeof(GLuint));

		fin.read((char*)&model->_modelDescription, sizeof(Model3D::ModelComponentDescription));
	}

	fin.close();

	return true;
}

void CADModel::readClassFile(const std::string& filename, std::map<std::string, std::string>& keyMap, std::string& defaultClass)
{
	const static char KEYWORD_DELIMITER = ';';

	// File management
	size_t delimiterIndex = std::string::npos;
	std::string currentLine, classKey, keyword, leftLine;
	std::list<std::string> keywords;
	std::stringstream line;
	std::ifstream inputStream;

	inputStream.open(filename.c_str());

	if (inputStream.fail()) return;

	while (!(inputStream >> std::ws).eof())
	{
		classKey = keyword = leftLine = "";
		keywords.clear();

		std::getline(inputStream, currentLine);

		if (currentLine.find(COMMENT_CHAR) != 0 && !currentLine.empty())		// Comment line
		{
			line.clear();
			line.str(currentLine);
			std::getline(line, classKey, '\t');

			if (classKey.empty())					// In case no tab alignment was followed
			{
				std::getline(line, classKey, ' ');
			}

			line >> leftLine;

			while (!leftLine.empty())
			{
				delimiterIndex = leftLine.find_first_of(KEYWORD_DELIMITER);

				if (delimiterIndex != std::string::npos)
				{
					keyword = leftLine.substr(0, delimiterIndex);
					leftLine = leftLine.substr(delimiterIndex + 1, leftLine.length() - delimiterIndex - 1);
				}
				else
				{
					keyword = leftLine;
					leftLine.clear();
				}

				if (!keyword.empty())
				{
					keywords.push_back(keyword);
				}
			}

			if (!classKey.empty() && keywords.empty())
			{
				defaultClass = classKey;
			}
			else
			{
				for (std::string& keywordString : keywords)
				{
					keyMap[keywordString] = classKey;
				}
			}
		}
	}

	inputStream.close();
}

void CADModel::setVAOData(ModelComponent* modelComp)
{
	VAO* vao = new VAO(true);

	modelComp->_topologyIndicesLength[RendEnum::IBO_POINT_CLOUD] = modelComp->_pointCloud.size();
	modelComp->_topologyIndicesLength[RendEnum::IBO_WIREFRAME] = modelComp->_wireframe.size();
	modelComp->_topologyIndicesLength[RendEnum::IBO_TRIANGLE_MESH] = modelComp->_triangleMesh.size();

	vao->setVBOData(modelComp->_geometry);
	vao->setIBOData(RendEnum::IBO_POINT_CLOUD, modelComp->_pointCloud);
	vao->setIBOData(RendEnum::IBO_WIREFRAME, modelComp->_wireframe);
	vao->setIBOData(RendEnum::IBO_TRIANGLE_MESH, modelComp->_triangleMesh);

	modelComp->_vao = vao;
}

void CADModel::sortPoints(GLuint pointBuffer, GLuint numPoints, const AABB& aabb)
{
	const GLuint pointCodeSSBO = this->calculateMortonCodes(pointBuffer, numPoints, aabb);
	const GLuint indicesBufferSSBO = this->sortFacesByMortonCode(pointCodeSSBO, numPoints);
	GLuint* indices = ComputeShader::readData(indicesBufferSSBO, GLuint());
	VertexGPUData* vertices = ComputeShader::readData(pointBuffer, VertexGPUData());
	std::vector<GLuint> bufferIndices = std::vector<GLuint>(indices, indices + numPoints);
	std::vector<VertexGPUData> verticesSupport(numPoints);

	for (int pointIdx = 0; pointIdx < numPoints; ++pointIdx)
	{
		verticesSupport.at(pointIdx) = vertices[indices[pointIdx]];
	}

	ComputeShader::updateReadBuffer(pointBuffer, verticesSupport.data(), numPoints, GL_STATIC_DRAW);

	glDeleteBuffers(1, &indicesBufferSSBO);
}

GLuint CADModel::sortFacesByMortonCode(const GLuint mortonCodes, unsigned numPoints)
{
	const unsigned numBits = 30;			// 10 bits per coordinate (3D)
	unsigned arraySize = numPoints;
	unsigned currentBits = 0;
	const int numGroups = ComputeShader::getNumGroups(arraySize);
	const int maxGroupSize = ComputeShader::getMaxGroupSize();
	GLuint* indices = new GLuint[arraySize];

	// Binary tree parameters
	const unsigned startThreads = unsigned(std::ceil(arraySize / 2.0f));
	const unsigned numExec = unsigned(std::ceil(std::log2(arraySize)));
	const unsigned numGroups2Log = unsigned(ComputeShader::getNumGroups(startThreads));
	unsigned numThreads = 0, iteration;

	// Fill indices array from zero to arraySize - 1
	for (int i = 0; i < arraySize; ++i) { indices[i] = i; }

	GLuint indicesBufferID_1, indicesBufferID_2, pBitsBufferID, nBitsBufferID, positionBufferID;
	indicesBufferID_1 = ComputeShader::setWriteBuffer(GLuint(), arraySize);
	indicesBufferID_2 = ComputeShader::setReadBuffer(indices, arraySize);					// Substitutes indicesBufferID_1 for the next iteration
	pBitsBufferID = ComputeShader::setWriteBuffer(GLuint(), arraySize);
	nBitsBufferID = ComputeShader::setWriteBuffer(GLuint(), arraySize);
	positionBufferID = ComputeShader::setWriteBuffer(GLuint(), arraySize);

	while (currentBits < numBits)
	{
		std::vector<GLuint> threadCount{ startThreads };
		threadCount.reserve(numExec);

		std::swap(indicesBufferID_1, indicesBufferID_2);							// indicesBufferID_2 is initialized with indices cause it's swapped here

		// FIRST STEP: BIT MASK, check if a morton code gives zero or one for a certain mask (iteration)
		unsigned bitMask = 1 << currentBits++;

		_bitMaskShader->bindBuffers(std::vector<GLuint> { mortonCodes, indicesBufferID_1, pBitsBufferID, nBitsBufferID });
		_bitMaskShader->use();
		_bitMaskShader->setUniform("arraySize", arraySize);
		_bitMaskShader->setUniform("bitMask", bitMask);
		_bitMaskShader->execute(numGroups, 1, 1, maxGroupSize, 1, 1);

		// SECOND STEP: build a binary tree with a summatory of the array
		_reduceShader->bindBuffers(std::vector<GLuint> { nBitsBufferID });
		_reduceShader->use();
		_reduceShader->setUniform("arraySize", arraySize);

		iteration = 0;
		while (iteration < numExec)
		{
			numThreads = threadCount[threadCount.size() - 1];

			_reduceShader->setUniform("iteration", iteration++);
			_reduceShader->setUniform("numThreads", numThreads);
			_reduceShader->execute(numGroups2Log, 1, 1, maxGroupSize, 1, 1);

			threadCount.push_back(std::ceil(numThreads / 2.0f));
		}

		// THIRD STEP: set last position to zero, its faster to do it in GPU than retrieve the array in CPU, modify and write it again to GPU
		_resetPositionShader->bindBuffers(std::vector<GLuint> { nBitsBufferID });
		_resetPositionShader->use();
		_resetPositionShader->setUniform("arraySize", arraySize);
		_resetPositionShader->execute(1, 1, 1, 1, 1, 1);

		// FOURTH STEP: build tree back to first level and compute position of each bit
		_downSweepShader->bindBuffers(std::vector<GLuint> { nBitsBufferID });
		_downSweepShader->use();
		_downSweepShader->setUniform("arraySize", arraySize);

		iteration = unsigned(threadCount.size()) - 2;
		while (iteration >= 0 && iteration < numExec)
		{
			_downSweepShader->setUniform("iteration", iteration);
			_downSweepShader->setUniform("numThreads", threadCount[iteration--]);
			_downSweepShader->execute(numGroups2Log, 1, 1, maxGroupSize, 1, 1);
		}

		_reallocatePositionShader->bindBuffers(std::vector<GLuint> { pBitsBufferID, nBitsBufferID, indicesBufferID_1, indicesBufferID_2 });
		_reallocatePositionShader->use();
		_reallocatePositionShader->setUniform("arraySize", arraySize);
		_reallocatePositionShader->execute(numGroups, 1, 1, maxGroupSize, 1, 1);
	}

	glDeleteBuffers(1, &indicesBufferID_1);
	glDeleteBuffers(1, &pBitsBufferID);
	glDeleteBuffers(1, &nBitsBufferID);
	glDeleteBuffers(1, &positionBufferID);

	delete[] indices;

	return indicesBufferID_2;
}

bool CADModel::writeToBinary()
{
	std::ofstream fout(_filename + ".bin", std::ios::out | std::ios::binary);
	if (!fout.is_open())
	{
		return false;
	}

	size_t numIndices;
	const size_t numModelComps = _modelComp.size();
	fout.write((char*)&numModelComps, sizeof(size_t));

	for (Model3D::ModelComponent* model : _modelComp)
	{
		const size_t numVertices = model->_geometry.size();
		fout.write((char*)&numVertices, sizeof(size_t));
		fout.write((char*)&model->_geometry[0], numVertices * sizeof(Model3D::VertexGPUData));

		const size_t numTriangles = model->_topology.size();
		fout.write((char*)&numTriangles, sizeof(size_t));
		fout.write((char*)&model->_topology[0], numTriangles * sizeof(Model3D::FaceGPUData));

		numIndices = model->_triangleMesh.size();
		fout.write((char*)&numIndices, sizeof(size_t));
		fout.write((char*)&model->_triangleMesh[0], numIndices * sizeof(GLuint));

		numIndices = model->_pointCloud.size();
		fout.write((char*)&numIndices, sizeof(size_t));
		fout.write((char*)&model->_pointCloud[0], numIndices * sizeof(GLuint));

		numIndices = model->_wireframe.size();
		fout.write((char*)&numIndices, sizeof(size_t));
		fout.write((char*)&model->_wireframe[0], numIndices * sizeof(GLuint));

		fout.write((char*)&model->_modelDescription, sizeof(Model3D::ModelComponentDescription));
	}

	fout.close();

	return true;
}

// [Convex hull]

CADModel::IndexTuple::IndexTuple(int a, int b, int c)
	: _orig(a), _dest(b), _triangleC(c)
{
	_hash = getEdgeHash();
}

CADModel::IndexTuple::~IndexTuple()
{
}

int CADModel::IndexTuple::getEdgeHash()
{
	int min = std::min(_orig, _dest);
	int max = std::max(_orig, _dest);

	return ((min + max) * (min + max + 1)) / 2 + max;
}

CADModel::IndexTuple& CADModel::IndexTuple::operator=(const IndexTuple& tuple)
{
	_orig = tuple._orig;
	_dest = tuple._dest;
	_triangleC = tuple._triangleC;
	_hash = tuple._hash;

	return *this;
}

bool CADModel::IndexTuple::operator==(const IndexTuple& tuple) const
{
	return _hash == tuple._hash;
}

bool CADModel::IndexTuple::operator!=(const IndexTuple& tuple) const
{
	return !(this->operator==(tuple));
}

bool CADModel::Face::operator==(const Face& face) const
{
	return  (face._indices.x == _indices.x && face._indices.y == _indices.y && face._indices.z == _indices.z) ||
			(face._indices.x == _indices.y && face._indices.y == _indices.z && face._indices.z == _indices.x) ||
			(face._indices.x == _indices.z && face._indices.y == _indices.x && face._indices.z == _indices.y);
}

// [Convex hull]

CADModel::ConvexHullProcessData::ConvexHullProcessData(ModelComponent* modelComp, int segmentA, int segmentB, int triangleC)
{
	Face face{ uvec3(segmentA, segmentB, triangleC), modelComp };

	_modelComp = modelComp;
	_pointIndex = std::unordered_set<int>{ segmentA, segmentB, triangleC };
	_triangles.push_back(Triangle3D(_modelComp->_geometry[segmentA]._position, _modelComp->_geometry[segmentB]._position, _modelComp->_geometry[triangleC]._position));
	_includedFace.insert(face);

	// Segment insertion
	IndexTuple segment1 = IndexTuple(segmentA, segmentB, triangleC), segment2 = IndexTuple(segmentA, triangleC, segmentB), segment3 = IndexTuple(segmentB, triangleC, segmentA);

	_boundaryCH.insert(segment1);
	_boundaryCH.insert(segment2);
	_boundaryCH.insert(segment3);
}

void CADModel::ConvexHullProcessData::updateData(int segmentA, int segmentB, int triangleC)
{
	Face face{ uvec3(segmentA, segmentB, triangleC), _modelComp };
	if (_includedFace.find(face) != _includedFace.end())
	{
		return;
	}

	IndexTuple edge1 = IndexTuple(segmentA, triangleC, segmentB), edge2 = IndexTuple(segmentB, triangleC, segmentA);
	int hash1 = edge1.getEdgeHash(), hash2 = edge2.getEdgeHash();

	_triangles.push_back(Triangle3D(_modelComp->_geometry[segmentA]._position, _modelComp->_geometry[segmentB]._position, _modelComp->_geometry[triangleC]._position));
	_includedFace.insert(face);

	if (_pointIndex.find(triangleC) == _pointIndex.end())					// First time this vertex appears.
	{
		_pointIndex.insert(triangleC);

		_boundaryCH.insert(edge1);
		_boundaryCH.insert(edge2);
	}
	else
	{
		auto segment1It = _boundaryCH.find(IndexTuple(segmentA, triangleC, segmentB));

		if (segment1It != _boundaryCH.end())
		{
			_boundaryCH.erase(segment1It);
		}
		else
		{
			_boundaryCH.insert(edge1);
		}

		auto segment2It = _boundaryCH.find(IndexTuple(segmentB, triangleC, segmentA));

		if (segment2It != _boundaryCH.end())
		{
			_boundaryCH.erase(segment2It);
		}
		else
		{
			_boundaryCH.insert(edge2);
		}
	}
}

/// [Structs]

size_t CADModel::IndexTupleHash::operator()(const IndexTuple& tuple) const
{
	return tuple._hash;
}

bool CADModel::IndexTupleEquals::operator()(const IndexTuple& a, const IndexTuple& b) const
{
	return a == b;
}

bool CADModel::FaceEquals::operator()(const Face& a, const Face& b) const
{
	return a == b;
}

size_t CADModel::FaceHash::operator()(const Face& face) const
{
	return face._indices.x + face._indices.y + face._indices.z;
}

void CADModel::Face::checkCCW()
{
	vec3 a = _modelComp->_geometry[_indices.x]._position, b = _modelComp->_geometry[_indices.y]._position, c = _modelComp->_geometry[_indices.z]._position;
	if ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y) < glm::epsilon<float>())
	{
		std::swap(_indices.x, _indices.y);
	}
}
