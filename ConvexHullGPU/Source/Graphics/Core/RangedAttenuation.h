#pragma once

#include "Graphics/Core/LightAttenuation.h"

/**
*	@file RangedAttenuation.h
*	@authors Alfonso L�pez Ruiz (alr00048@red.ujaen.es)
*	@date 07/17/2019
*/

/**
*	@brief Applicator for an attenuationa which is applied in a range of distances.
*/
class RangedAttenuation: public LightAttenuation
{
public:
	/**
	*	@brief Applies the uniform variables associated to an specific attenuation formula.
	*	@param light Light from where we get the parameters.
	*	@param shader Shader where we specity the uniform variables.
	*/
	virtual void applyAttenuation(Light* light, RenderingShader* shader);

	/**
	*	@brief Applies the uniform variables associated to an specific attenuation formula.
	*	@param light Light from where we get the parameters.
	*	@param shader Shader where we specity the uniform variables.
	*/
	virtual void applyAttenuation4ColouredPoints(Light* light, RenderingShader* shader);
};

