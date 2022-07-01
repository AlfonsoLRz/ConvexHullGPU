#include "stdafx.h"
#include "DrawRay3D.h"

#include "Geometry/3D/TriangleMesh.h"

/// [Static members inicialization]

const float DrawRay3D::EXCEED_BOUNDARIES = 10000.0f;

/// [Public methods]

DrawRay3D::DrawRay3D(Ray3D* ray, const mat4& modelMatrix) :
	Model3D(modelMatrix), _ray(ray), _vao(nullptr)
{
}

DrawRay3D::~DrawRay3D()
{
	delete _vao;
}

void DrawRay3D::drawAsLines(RenderingShader* shader, const RendEnum::RendShaderTypes shaderType, std::vector<mat4>& matrix)
{
	if (_vao)
	{
		this->setShaderUniforms(shader, shaderType, matrix);

		_vao->drawObject(RendEnum::IBO_WIREFRAME, GL_LINES, 2);
	}
}

void DrawRay3D::drawAsPoints(RenderingShader* shader, const RendEnum::RendShaderTypes shaderType, std::vector<mat4>& matrix)
{
	if (_vao)
	{
		this->setShaderUniforms(shader, shaderType, matrix);

		_vao->drawObject(RendEnum::IBO_POINT_CLOUD, GL_POINTS, 2);
	}
}

void DrawRay3D::drawAsTriangles(RenderingShader* shader, const RendEnum::RendShaderTypes shaderType, std::vector<mat4>& matrix)
{
	this->drawAsLines(shader, shaderType, matrix);
}

void DrawRay3D::drawAsTriangles4Fog(RenderingShader* shader, const RendEnum::RendShaderTypes shaderType, std::vector<mat4>& matrix)
{
	this->drawAsLines(shader, shaderType, matrix);
}

void DrawRay3D::drawAsTriangles4Shadows(RenderingShader* shader, const RendEnum::RendShaderTypes shaderType, std::vector<mat4>& matrix)
{
	this->drawAsLines(shader, shaderType, matrix);
}

void DrawRay3D::drawCapturedPoints(RenderingShader* shader, const RendEnum::RendShaderTypes shaderType, std::vector<mat4>& matrix)
{
}

bool DrawRay3D::load(const mat4& modelMatrix)
{
	if (_loaded || !_ray)
	{
		return false;
	}

	this->createVAO();
	this->_loaded = true;

	return true;
}

/// [Protected methods]

void DrawRay3D::createVAO()
{
	std::vector<vec4> points{ vec4(_ray->getOrigin(), 1.0f), vec4(_ray->getPoint(EXCEED_BOUNDARIES), 1.0f) };
	std::vector<GLuint> wireframe{ 0, 1 };

	_vao = new VAO();
	_vao->setVBOData(RendEnum::VBO_POSITION, points);
	_vao->setIBOData(RendEnum::IBO_POINT_CLOUD, wireframe);
	_vao->setIBOData(RendEnum::IBO_WIREFRAME, wireframe);
}
