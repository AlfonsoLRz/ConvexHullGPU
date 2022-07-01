#include "stdafx.h"
#include "Image.h"

/// [Public methods]

Image::Image(const std::string& filename) :
	_depth (4)				// PNG depth
{
	unsigned error = lodepng::decode(_image, _width, _height, filename.c_str());

	if (error)
	{
		std::cout << "The image " << filename << " couldn't be loaded by lodepng" << std::endl;

		_width = _height = _depth = 0;
	}

	this->flipImageVertically(_image, _width, _height, _depth);				// By default it's flipped
}

Image::Image(unsigned char* image, const uint16_t width, const uint16_t height, const uint8_t depth) :
	_width(width), _height(height), _depth(depth)
{
	if (image)
	{
		const int size = width * height * depth;
		_image = std::vector<unsigned char>(image, image + size);
	}
	else
	{
		std::cout << "Empty image!" << std::endl;

		_width = _height = _depth = 0;
	}
}

Image::~Image()
{
}

void Image::flipImageVertically(std::vector<unsigned char>& image, const uint16_t width, const uint16_t height, const uint8_t depth)
{
	int rowSize = width * depth;
	unsigned char* bits = image.data();
	unsigned char* tempBuffer = new unsigned char[rowSize];

	for (int i = 0; i < (height / 2); ++i)
	{
		unsigned char* source = bits + i * rowSize;
		unsigned char* target = bits + (height - i - 1) * rowSize;

		memcpy(tempBuffer, source, rowSize);					// Swap with help of temporary buffer
		memcpy(source, target, rowSize);
		memcpy(target, tempBuffer, rowSize);
	}

	delete[] tempBuffer;
}

void Image::flipImageVertically()
{
	Image::flipImageVertically(_image, _width, _height, _depth);
}