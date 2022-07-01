#include "stdafx.h"
#include "TriangleSet.h"

// [Public methods]

TriangleSet::TriangleSet() : Model3D()
{
}

TriangleSet::~TriangleSet()
{
}

void TriangleSet::clear()
{
	_modelComp[0]->releaseMemory();
}

void TriangleSet::insertFaces(std::vector<VertexGPUData>* vertices, FaceGPUData* faces, GLuint numFaces)
{
	for (int face = 0; face < numFaces; ++face)
	{
		unsigned numVertices = _modelComp[0]->_geometry.size();

		_modelComp[0]->_geometry.push_back(vertices->at(faces[face]._vertices.x));
		_modelComp[0]->_geometry.push_back(vertices->at(faces[face]._vertices.y));
		_modelComp[0]->_geometry.push_back(vertices->at(faces[face]._vertices.z));

		_modelComp[0]->_triangleMesh.push_back(numVertices + 0);
		_modelComp[0]->_triangleMesh.push_back(numVertices + 1);
		_modelComp[0]->_triangleMesh.push_back(numVertices + 2);
		_modelComp[0]->_triangleMesh.push_back(Model3D::RESTART_PRIMITIVE_INDEX);
	}
}

bool TriangleSet::load(const mat4& modelMatrix)
{
	this->setVAOData();

	return true;
}
