#pragma once

#include "tsdfBase.h"

class TSDFTexture : public TSDFBase
{
public:
	TSDFTexture();

	virtual ~TSDFTexture();

	virtual void UpdateVoxel(Voxel &voxel, cv::Mat &inv_pose, cv::Mat &depth_img, cv::Mat &rgb_img);

	virtual void UpdateVoxel(std::shared_ptr<Voxel> voxel, cv::Mat & inv_pose, cv::Mat &depth_img, cv::Mat &rgb_img);

	virtual void UpdateVolume(cv::Mat &depth_img, cv::Mat &rgb_img, cv::Mat &inv_pose);

	virtual void InitVolume();

	virtual void ResetVoxel(int index);

	virtual void Save();

	virtual void GetRGB(unsigned int index, uchar &r, uchar &g, uchar &b);

	void SetTheta(float theta);

	void GetTheta(float &theta);

	void UpdateVoxel(unsigned int index, cv::Mat &depth_img, cv::Mat &rgb_img, cv::Mat &inv_pose, cv::Point3f &voxel_pos);

	void CalcAndUpdate(unsigned int index, cv::Mat &depth_img, cv::Mat &rgb_img, cv::Mat &inv_pose, cv::Point3f &voxel_pos);

protected:
	float mTheta;

	std::vector<ColorVoxel> mColorVolume;
};