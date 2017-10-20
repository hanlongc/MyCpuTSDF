#pragma once

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>

#ifdef _WIN32
#include <Windows.h>
#else
#include <sys/time.h>
#endif


#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/ply_io.h>

#include "defs.h"
#include "tsdfGeometry.h"
#include "tsdfTexture.h"
#include "marchingCube.h"
#include "textureGenerator.h"
#include "tsdfVolumetricFusion.h"

class Application
{
public:
	Application(int argc, char** argv);
	~Application();

	void Exec();

	void Run();

	void TestTexture();

	void Texture();

private:
	void GetDefaultData(std::vector<cv::Mat> &depth_imgs, std::vector<cv::Mat> &rgb_imgs, std::vector<TUMPose> &tum_poses);

#ifdef _WIN32
	float TimeCost(SYSTEMTIME &sys_prev, SYSTEMTIME &sys_later);
#endif

	void FromQuanternion2Matrix(TUMPose &tum_pose, cv::Mat &camera_pose);

	void InitDefaultSetting();

	void SetParameter(std::shared_ptr<TSDFBase> tsdf);

	void GetDefaultSetting();

	std::string GetDataByName(std::string node_name);

	void MatrixXMatrix(cv::Mat left, cv::Mat right, cv::Mat &output);

	void TestGenerateTexture(pcl::PointCloud<pcl::PointXYZRGB>::Ptr mesh_cloud, std::shared_ptr<TSDFBase> tsdf_ptr);

	void TestStitchAndUpdate(cv::Mat &texture, std::vector<cv::Point2i> &coord, std::vector<cv::Mat> &textures, int curr_col, std::vector<int> &point_idx);

	void TestSaveResult(pcl::PointCloud<pcl::PointXYZRGB>::Ptr mesh_cloud, std::vector<cv::Point2f> &tex_coord);

	void TestSaveColorVolumeResult(pcl::PointCloud<pcl::PointXYZRGB>::Ptr mesh_cloud);

private:
	int mArgc;
	char** mArgv;

	float mFactor;
	float mFX;
	float mFY;
	float mCX;
	float mCY;

	float mTruncateDist;
	float mTheta;

	int mResX;
	int mResY;
	int mResZ;

	float mSizeX;
	float mSizeY;
	float mSizeZ;

	int mFrameNum;

	std::string mVoxelType;

	std::shared_ptr<TSDFBase> mTsdfPtr;

	float mRangeMin;
	float mRangeMax;

	float mMaxWeight;

	std::string mSaveTsdf;

	float mSaveThresh;

	std::string mToMesh;
};