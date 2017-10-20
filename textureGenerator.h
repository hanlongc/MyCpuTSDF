#pragma once

#include <pcl/common/common.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/search/kdtree.h>

#include "tsdfBase.h"

class TextureGenerator
{
public:
	TextureGenerator();

	~TextureGenerator();

	void Generate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr mesh_cloud, std::shared_ptr<TSDFBase> tsdf_ptr);

	void Generate();

	void SetColorResolution(int res_x, int res_y, int res_z);

	void SetColorSize(float size_x, float size_y, float size_z);

	void SetGeometryResolution(int res_x, int res_y, int res_z);

	void SetGeometrySize(float size_x, float size_y, float size_z);

private:
	void GenerateTexture(pcl::PointCloud<pcl::PointXYZRGB>::Ptr mesh_cloud, 
						 pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud, 
						 cv::Mat &texture, 
						 std::vector<cv::Point2f> &tex_coords);

	void Save(pcl::PointCloud<pcl::PointXYZRGB>::Ptr mesh_cloud, 
			  cv::Mat &texture,
			  std::vector<cv::Point2f> &tex_coords);

	void StitchAndUpdate(cv::Mat &texture, 
						 std::vector<cv::Point2i> &coords, 
						 std::vector<cv::Mat> &textures, 
						 int curr_col, 
						 std::vector<int> &point_idx);

private:
	int mCResX;
	int mCResY;
	int mCResZ;
	int mGResX;
	int mGResY;
	int mGResZ;

	float mCSizeX;
	float mCSizeY;
	float mCSizeZ;
	float mGSizeX;
	float mGSizeY;
	float mGSizeZ;
};