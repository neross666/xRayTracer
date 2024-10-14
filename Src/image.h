#pragma once
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>

#include "geometry.h"

class Image
{
public:
	struct ImageIdx
	{
		int i;
		int j;
	};

	Image(uint32_t width, uint32_t height) : width(width), height(height)
	{
		pixels.resize(width * height);

		for (int i = 0; i < height; ++i) {
			for (int j = 0; j < width; ++j) {
				elems.emplace_back(ImageIdx{ i,j });
			}
		}
	}

	uint32_t getWidth() const { return width; }
	uint32_t getHeight() const { return height; }


	const std::vector<ImageIdx>& getImageIdx() const
	{
		return elems;
	}

	Vec3f getPixel(uint32_t i, uint32_t j) const
	{
		const uint32_t idx = getIndex(i, j);
		return pixels[idx];
	}

	void addPixel(uint32_t i, uint32_t j, const Vec3f& rgb)
	{
		const uint32_t idx = getIndex(i, j);
		pixels[idx] += rgb;
	}

	void setPixel(uint32_t i, uint32_t j, const Vec3f& rgb)
	{
		const uint32_t idx = getIndex(i, j);
		pixels[idx] = rgb;
	}

	Image& operator*=(const Vec3f& rgb)
	{
		for (int i = 0; i < height; ++i) {
			for (int j = 0; j < width; ++j) {
				const Vec3f c = getPixel(i, j);
				setPixel(i, j, c * rgb);
			}
		}
		return *this;
	}

	Image& operator/=(const Vec3f& rgb)
	{
		for (int i = 0; i < height; ++i) {
			for (int j = 0; j < width; ++j) {
				const Vec3f c = getPixel(i, j);
				setPixel(i, j, c / rgb);
			}
		}
		return *this;
	}

	void gammaCorrection(const float gamma)
	{
		for (int i = 0; i < height; ++i) {
			for (int j = 0; j < width; ++j) {
				auto idx = getIndex(i, j);
				pixels[idx][0] = std::pow(pixels[idx][0], 1.0f / gamma);
				pixels[idx][1] = std::pow(pixels[idx][1], 1.0f / gamma);
				pixels[idx][2] = std::pow(pixels[idx][2], 1.0f / gamma);
			}
		}
	}

	void writePPM(const std::string& filename)
	{
		std::ofstream file(filename);

		file << "P3" << std::endl;
		file << width << " " << height << std::endl;
		file << "255" << std::endl;

		for (int i = 0; i < height; ++i) {
			for (int j = 0; j < width; ++j) {
				const Vec3f rgb = getPixel(i, j);
				const uint32_t R =
					std::clamp(static_cast<uint32_t>(255.0f * rgb[0]), 0u, 255u);
				const uint32_t G =
					std::clamp(static_cast<uint32_t>(255.0f * rgb[1]), 0u, 255u);
				const uint32_t B =
					std::clamp(static_cast<uint32_t>(255.0f * rgb[2]), 0u, 255u);
				file << R << " " << G << " " << B << std::endl;
			}
		}

		file.close();
	}

	cv::Mat writeMat()
	{
		cv::Mat mat(height, width, CV_8UC3);
		for (int i = 0; i < height; ++i) {
			auto prow = mat.ptr<uchar>(i);
			for (int j = 0; j < width; ++j) {
				const Vec3f rgb = getPixel(i, j);
				const uint8_t R =
					std::clamp(static_cast<uint32_t>(255.0f * rgb[0]), 0u, 255u);
				const uint8_t G =
					std::clamp(static_cast<uint32_t>(255.0f * rgb[1]), 0u, 255u);
				const uint8_t B =
					std::clamp(static_cast<uint32_t>(255.0f * rgb[2]), 0u, 255u);

				prow[3 * j] = B;
				prow[3 * j + 1] = G;
				prow[3 * j + 2] = R;
			}
		}
		return mat;
	}


private:
	uint32_t getIndex(uint32_t i, uint32_t j) const
	{
		return j + width * i;
	}

private:
	uint32_t width;
	uint32_t height;
	std::vector<Vec3f> pixels;
	std::vector<ImageIdx> elems;
};
