#pragma once

#include <pcl/surface/marching_cubes.h>
#include <pcl/surface/impl/marching_cubes.hpp>

#include "tsdfGeometry.h"

class MarchingCube : public pcl::MarchingCubes<pcl::PointXYZRGB>
{
	using pcl::SurfaceReconstruction<pcl::PointXYZRGB>::input_;
	using pcl::SurfaceReconstruction<pcl::PointXYZRGB>::tree_;
	using pcl::MarchingCubes<pcl::PointXYZRGB>::grid_;
	using pcl::MarchingCubes<pcl::PointXYZRGB>::res_x_;
	using pcl::MarchingCubes<pcl::PointXYZRGB>::res_y_;
	using pcl::MarchingCubes<pcl::PointXYZRGB>::res_z_;
	using pcl::MarchingCubes<pcl::PointXYZRGB>::min_p_;
	using pcl::MarchingCubes<pcl::PointXYZRGB>::max_p_;

public:
	MarchingCube();

	~MarchingCube();

	void SetMinWeight(float min_w);

	void GetMinWeight(float &min_w);

	void SetTsdfPtr(std::shared_ptr<TSDFBase> tsdf_ptr);

	void GetTsdfPtr(std::shared_ptr<TSDFBase> tsdf_ptr);

protected:
	virtual void voxelizeData();

	void interpolateEdge(Eigen::Vector3f &p1, Eigen::Vector3f &p2, float val_p1, float val_p2, Eigen::Vector3f &out_pt, Eigen::Vector3i &out_color);

	void createSurface(std::vector<float> &leaf_node, Eigen::Vector3i &index_3d, pcl::PointCloud<pcl::PointXYZRGB> &cloud);

	void performReconstruction(pcl::PolygonMesh &output);

	void SetInputTSDFCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tsdf_cloud);

private:
	float mMinWeight;

	std::shared_ptr<TSDFBase> mTsdfPtr;
};