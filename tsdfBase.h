#pragma once

#include <iostream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "voxel.h"
#include "defs.h"

class TSDFBase
{
public:
	TSDFBase();

	virtual ~TSDFBase();

	void SetResolution(int res_x, int res_y, int res_z);

	void GetResolution(int &res_x, int &res_y, int &res_z);

	void SetSize(float size_x, float size_y, float size_z);

	void GetSize(float &size_x, float &size_y, float &size_z);

	void SetCamera(float fx, float fy, float cx, float cy, float factor);

	void GetCamera(float &fx, float &fy, float &cx, float &cy, float &factor);

	void SetSensorRange(float min_dist, float max_dist);

	void GetSensorRange(float &min_dist, float &max_dist);

	void SetMaxWeight(float max_weight);

	void GetMaxWeight(float &max_weight);

	void CalcNormalMap(cv::Mat &depth_img, cv::Mat &rgb_img, cv::Mat &normal_map);

	void FromWorld2Camera(cv::Point3f in_pt, cv::Point3f &out_pt, cv::Mat &transform);

	void FromCamera2Image(cv::Point3f in_pt, cv::Point2i &out_pt);

	void FromImage2Camera(cv::Point3f in_pt, cv::Point3f &out_pt);

	void CrossProduct(cv::Point3f vector1, cv::Point3f vector2, cv::Point3f &vector_out);

	void Normalize(cv::Point3f vector_in, cv::Point3f &vector_out);

	float DotProduct(cv::Point3f vector1, cv::Point3f vector2);

	virtual void UpdateVoxel(Voxel &voxel, cv::Mat &inv_pose, cv::Mat &depth_img, cv::Mat &rgb_img);

	virtual void UpdateVoxel(std::shared_ptr<Voxel> voxel, cv::Mat &inv_pose, cv::Mat &depth_img, cv::Mat &rgb_img);

	virtual void UpdateVolume(cv::Mat &depth_img, cv::Mat &rgb_img, cv::Mat &inv_pose);

	virtual void InitVolume();

	virtual void ResetVoxel(unsigned int index);

	virtual void Save();

	virtual void SaveVolumeData();

	virtual void GetTSDFCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tsdf_cloud);

	virtual float GetWeight(unsigned int index);

	virtual float GetTsdfValue(unsigned int index);

	virtual void GetRGB(unsigned int index, uchar &r, uchar &g, uchar &b);

protected:
	int mResX;
	int mResY;
	int mResZ;

	float mSizeX;
	float mSizeY;
	float mSizeZ;

	Camera mCamera;

	float mRangeMin;
	float mRangeMax;

	float mMaxWeight;
};