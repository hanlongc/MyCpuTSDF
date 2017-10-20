#pragma once

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

class Association
{
public:
	double mRGBTimestamp;
	double mDepthTimestamp;

	std::string mRGBImgName;
	std::string mDepthImgName;
};

class TUMPose
{
public:
	double mTS;
	double mTX;
	double mTY;
	double mTZ;
	double mQX;
	double mQY;
	double mQZ;
	double mQW;
};

/// Default size and resolution.
//const double DEFAULT_SIZE = 2.0;
//const int DEFAULT_RES = 200;
//
///// Voxel type.
//enum{ GEOMETRY=0, COLOR };
//
///// Default data frames.
//const int FRAME_NUM = 10;
//
///// Default truncate distance.
//const double DEFAULT_TRUNCATE_DIST = 0.06;

class Camera
{
public:
	Camera(float fx, float fy, float cx, float cy, float factor)
		: mFX( fx )
		, mFY( fy )
		, mCX( cx )
		, mCY( cy )
		, mFactor( factor )
	{}

	float mFX;
	float mFY;
	float mCX;
	float mCY;
	float mFactor;
};