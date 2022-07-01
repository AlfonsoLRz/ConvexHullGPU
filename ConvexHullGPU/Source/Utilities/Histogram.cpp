#include "stdafx.h"
#include "Histogram.h"

// [Public methods]

Histogram::Histogram(std::vector<float>* data) : _data(data)
{
}

void Histogram::buildHistogram(unsigned numBins, float minValue, float maxValue, bool density)
{
	_histogram.resize(numBins);
	_boundary = vec2(minValue, maxValue);
	std::fill(_histogram.begin(), _histogram.end(), 0);

	for (float value: *_data)
	{
		float scaledValue = (value - minValue) / (maxValue - minValue) * (numBins - 1.0f);
		++_histogram[std::round(scaledValue)];
	}

	if (density)
	{
		for (int histIndex = 0; histIndex < numBins; ++histIndex)
		{
			_histogram[histIndex] /= _data->size();
		}
	}
}

void Histogram::exportLatex(const std::string& filename)
{
	std::string latexLine = "{ ";

	for (int histIndex = 0; histIndex < _histogram.size(); ++histIndex)
	{
		latexLine += "(" + std::to_string((histIndex * (_boundary.y - _boundary.x) / _histogram.size()) + _boundary.x) + ", " + std::to_string(_histogram[histIndex]) + ") ";
	}

	latexLine += "}";

	{
		std::ofstream out(filename);
		out << latexLine;
		out.close();
	}
}

// [Static methods]