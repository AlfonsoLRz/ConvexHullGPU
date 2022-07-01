#pragma once

/**
*	@file Image.h
*	@authors Alfonso López Ruiz (alr00048@red.ujaen.es)
*	@date 08/04/2019
*/

/**
*	@brief Image wrapper for a set of bits with associated information.
*/
class Image
{
protected:
	std::vector<unsigned char> _image;					//!< Image bits
	unsigned _width, _height, _depth;					//!< Image dimensions

public:
	/**
	*	@brief Constructor of image from a filename from the system.
	*	@param filename Path of image.
	*/
	Image(const std::string& filename);

	/**
	*	@brief Constructor of already loaded image, so we just acts as a wrapper.
	*	@param image Array of image bits.
	*	@param width Image width.
	*	@param height Image height.
	*	@param depth Number of channels.
	*/
	Image(unsigned char* image, const uint16_t width, const uint16_t height, const uint8_t depth);

	/**
	*	@brief Destructor.
	*/
	~Image();

	/**
	*	@return Array of bits from image.
	*/
	unsigned char* bits() { return _image.data(); }

	/**
	*	@brief Flips the image so that bottom side turns into the upper side.
	*	@param image Array of image bits.
	*	@param width Image width.
	*	@param height Image height.
	*	@param depth Number of channels.
	*/
	void flipImageVertically();

	/**
	*	@return Number of channels of image.
	*/
	int getDepth() const { return _depth; }

	/**
	*	@return Height of image.
	*/
	int getHeight() const { return _height; }

	/**
	*	@return Width of image.
	*/
	int getWidth() const { return _width; }

	// ----------- Static methods ------------

	/**
	*	@brief Flips the image so that bottom side turns into the upper side.
	*	@param image Array of image bits.
	*	@param width Image width.
	*	@param height Image height.
	*	@param depth Number of channels.
	*/
	static void flipImageVertically(std::vector<unsigned char>& image, const uint16_t width, const uint16_t height, const uint8_t depth);
};

