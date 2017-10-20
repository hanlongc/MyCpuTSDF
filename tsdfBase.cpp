#include "tsdfBase.h"

///////////////////////////////////////////////////////////////////////////////////////////////////

TSDFBase::TSDFBase()
	: mResX( 0 )
	, mResY( 0 )
	, mResZ( 0 )
	, mSizeX( 0.0 )
	, mSizeY( 0.0 )
	, mSizeZ( 0.0 )
	, mCamera( Camera(0.0, 0.0, 0.0, 0.0, 0.0) )
	, mRangeMin( 0.0 )
	, mRangeMax( 0.0 )
	, mMaxWeight( 0.0 )
{

}

///////////////////////////////////////////////////////////////////////////////////////////////////

TSDFBase::~TSDFBase()
{

}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFBase::SetResolution(int res_x, int res_y, int res_z)
{
	mResX = res_x;
	mResY = res_y;
	mResZ = res_z;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFBase::GetResolution(int &res_x, int &res_y, int &res_z)
{
	res_x = mResX;
	res_y = mResY;
	res_z = mResZ;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFBase::SetSize(float size_x, float size_y, float size_z)
{
	mSizeX = size_x;
	mSizeY = size_y;
	mSizeZ = size_z;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFBase::GetSize(float &size_x, float &size_y, float &size_z)
{
	size_x = mSizeX;
	size_y = mSizeY;
	size_z = mSizeZ;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFBase::UpdateVoxel(Voxel &voxel, cv::Mat &inv_pose, cv::Mat &depth_img, cv::Mat &rgb_img)
{
	std::cout<<"This is the base tsdf class member function."<<std::endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFBase::UpdateVoxel(std::shared_ptr<Voxel> voxel, cv::Mat &inv_pose, cv::Mat &depth_img, cv::Mat &rgb_img)
{
	std::cout<<"This is the base tsdf class member function with ptr."<<std::endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFBase::UpdateVolume(cv::Mat &depth_img, cv::Mat &rgb_img, cv::Mat &inv_pose)
{
	/// TODO: Should be inherited.
	std::cout<<"TSDFBase"<<std::endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFBase::CalcNormalMap(cv::Mat &depth_img, cv::Mat &rgb_img, cv::Mat &normal_map)
{

}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFBase::InitVolume()
{

}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFBase::ResetVoxel(unsigned int index)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFBase::SetCamera(float fx, float fy, float cx, float cy, float factor)
{
	mCamera.mFX = fx;
	mCamera.mFY = fy;
	mCamera.mCX = cx;
	mCamera.mCY = cy;
	mCamera.mFactor = factor;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFBase::GetCamera(float &fx, float &fy, float &cx, float &cy, float &factor)
{
	fx = mCamera.mFX;
	fy = mCamera.mFY;
	cx = mCamera.mCX;
	cy = mCamera.mCY;
	factor = mCamera.mFactor;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFBase::FromWorld2Camera(cv::Point3f in_pt, cv::Point3f &out_pt, cv::Mat &transform)
{
	out_pt.x = in_pt.x * transform.at<float>(0, 0) + in_pt.y * transform.at<float>(0, 1) + in_pt.z * transform.at<float>(0, 2) + transform.at<float>(0, 3);
	out_pt.y = in_pt.x * transform.at<float>(1, 0) + in_pt.y * transform.at<float>(1, 1) + in_pt.z * transform.at<float>(1, 2) + transform.at<float>(1, 3);
	out_pt.z = in_pt.x * transform.at<float>(2, 0) + in_pt.y * transform.at<float>(2, 1) + in_pt.z * transform.at<float>(2, 2) + transform.at<float>(2, 3);

	/*out_pt.x = in_pt.x * transform.at<float>(0, 0) + in_pt.y * transform.at<float>(1, 0) + in_pt.z * transform.at<float>(2, 0) + transform.at<float>(0, 3);
	out_pt.y = in_pt.x * transform.at<float>(0, 1) + in_pt.y * transform.at<float>(1, 1) + in_pt.z * transform.at<float>(2, 1) + transform.at<float>(1, 3);
	out_pt.z = in_pt.x * transform.at<float>(0, 2) + in_pt.y * transform.at<float>(1, 2) + in_pt.z * transform.at<float>(2, 2) + transform.at<float>(2, 3);*/
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFBase::FromCamera2Image(cv::Point3f in_pt, cv::Point2i &out_pt)
{
	float u = in_pt.x * mCamera.mFX / in_pt.z + mCamera.mCX;
	float v = in_pt.y * mCamera.mFY / in_pt.z + mCamera.mCY;

	int uFloor = (int)(u);
	int vFloor = (int)(v);

	if ( (u-uFloor)>=0.5 )
		out_pt.x = uFloor+1;
	else
		out_pt.x = uFloor;

	if ( (v-vFloor)>=0.5 )
		out_pt.y = vFloor+1;
	else
		out_pt.y = vFloor;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFBase::FromImage2Camera(cv::Point3f in_pt, cv::Point3f &out_pt)
{
	out_pt.z = in_pt.z / mCamera.mFactor;
	out_pt.x = (in_pt.x - mCamera.mCX) * out_pt.z / mCamera.mFX;
	out_pt.y = (in_pt.y - mCamera.mCY) * out_pt.z / mCamera.mFY;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFBase::SetSensorRange(float min_dist, float max_dist)
{
	mRangeMin = min_dist;
	mRangeMax = max_dist;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFBase::GetSensorRange(float &min_dist, float &max_dist)
{
	min_dist = mRangeMin;
	max_dist = mRangeMax;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFBase::SetMaxWeight(float max_weight)
{
	mMaxWeight = max_weight;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFBase::GetMaxWeight(float &max_weight)
{
	max_weight = mMaxWeight;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFBase::Save()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFBase::GetTSDFCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tsdf_cloud)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float TSDFBase::GetWeight(unsigned int index)
{
	return -1.0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float TSDFBase::GetTsdfValue(unsigned int index)
{
	return -1.0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFBase::GetRGB(unsigned int index, uchar &r, uchar &g, uchar &b)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFBase::CrossProduct(cv::Point3f in_vector1, cv::Point3f in_vector2, cv::Point3f &out_vector)
{
	float y1z2 = in_vector1.y * in_vector2.z;
	float z1y2 = in_vector1.z * in_vector2.y;
	float z1x2 = in_vector1.z * in_vector2.x;
	float x1z2 = in_vector1.x * in_vector2.z;
	float x1y2 = in_vector1.x * in_vector2.y;
	float y1x2 = in_vector1.y * in_vector2.x;

	out_vector.x = y1z2 - z1y2;
	out_vector.y = z1x2 - x1z2;
	out_vector.z = x1y2 - y1x2;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFBase::Normalize(cv::Point3f in_vector, cv::Point3f &out_vector)
{
	if ( in_vector.x==0.0 && in_vector.y==0.0 && in_vector.z==0.0 )
		return;

	float xx = in_vector.x * in_vector.x;
	float yy = in_vector.y * in_vector.y;
	float zz = in_vector.z * in_vector.z;
	float sum = xx + yy + zz;

	out_vector.x = std::sqrt( xx / sum );
	out_vector.y = std::sqrt( yy / sum );
	out_vector.z = std::sqrt( zz / sum );
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float TSDFBase::DotProduct(cv::Point3f in_vector1, cv::Point3f in_vector2)
{
	float x1x2 = in_vector1.x * in_vector2.x;
	float y1y2 = in_vector1.y * in_vector2.y;
	float z1z2 = in_vector1.z * in_vector2.z;

	return (x1x2 + y1y2 + z1z2);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFBase::SaveVolumeData()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

