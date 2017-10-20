#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class Voxel
{
public:
	float mRed;
	float mGreen;
	float mBlue;

	float mWeight;

	cv::Point3f mPositon;
};

class GeometryVoxel
{
public:
	float mRed;
	float mGreen;
	float mBlue;

	float mWeight;

	cv::Point3f mPos;

	float mTsdfValue;
};

class ColorVoxel
{
public:
	uchar mRed;
	uchar mGreen;
	uchar mBlue;

	float mWeight;
};

class SmallGeometryVoxel
{
public:
	uchar mRed;
	uchar mGreen;
	uchar mBlue;
	
	uchar mWeight;

	float mTsdfValue;
};