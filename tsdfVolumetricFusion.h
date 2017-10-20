#pragma once

#include <algorithm>
#include <fstream>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>

#include "tsdfBase.h"

class TSDFVolumetricFusion : public TSDFBase
{
public:
	TSDFVolumetricFusion();

	virtual ~TSDFVolumetricFusion();

	virtual void UpdateVolume(cv::Mat &depth_img, cv::Mat &rgb_img, cv::Mat &inv_pose);

	virtual void InitVolume();

	virtual void ResetVoxel(unsigned int index);

	virtual void Save();

	virtual void SaveVolumeData();

	virtual void GetTSDFCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tsdf_cloud);

	virtual float GetWeight(unsigned int index);

	virtual float GetTsdfValue(unsigned int index);

	virtual void GetRGB(unsigned int index, uchar &r, uchar &g, uchar &b);

	void SetTruncateDist(float truncate_dist);

	void GetTruncateDist(float &truncate_dist);

	void SetSaveThresh(float save_thresh);

	void GetSaveThresh(float &save_thresh);

	void UpdateVoxel(unsigned int index, cv::Mat &depth_img, cv::Mat &rgb_img, cv::Mat &inv_pose, cv::Point3f &voxel_pos);

	void CalcAndUpdate(unsigned int index, cv::Mat &depth_img, cv::Mat &rgb_img, cv::Mat &inv_pose, cv::Point3f &voxel_pos);

	void GetValidVolume(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tsdf_volume);

	void GetVoxel(int index, SmallGeometryVoxel &voxel);

protected:
	std::vector<SmallGeometryVoxel> mGeoVolume;

	float mTruncateDist;
	float mSaveThresh;
};