#include "tsdfGeometry.h"

///////////////////////////////////////////////////////////////////////////////////////////////////

TSDFGeometry::TSDFGeometry()
	: TSDFBase()
	, mTruncateDist( 0.0 )
	, mSaveThresh( 0.0 )
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

TSDFGeometry::~TSDFGeometry()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFGeometry::UpdateVoxel(Voxel &voxel, cv::Mat &inv_pose, cv::Mat &depth_img, cv::Mat &rgb_img)
{
	std::cout<<"This the child class tsdf geometry member function."<<std::endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFGeometry::UpdateVoxel(std::shared_ptr<Voxel> voxel, cv::Mat &inv_pose, cv::Mat &depth_img, cv::Mat &rgb_img)
{
	std::cout<<"This is the child class tsdf geometry member function with ptr."<<std::endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFGeometry::UpdateVolume(cv::Mat &depth_img, cv::Mat &rgb_img, cv::Mat &inv_pose)
{
	for (int x = 0; x < mResX; ++x)
	{
		for (int y = 0; y < mResY; ++y)
		{
			for (int z = 0; z < mResZ; ++z)
			{
				unsigned int index = x + y * mResX + z * mResX * mResY;

				UpdateVoxel(index, depth_img, rgb_img, inv_pose);
				//CalcAndUpdate(index, depth_img, rgb_img, inv_pose);
			}//end inner for
		}//end for
	}//end for
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFGeometry::SetTruncateDist(float truncate_dist)
{
	mTruncateDist = truncate_dist;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFGeometry::GetTruncateDist(float &truncate_dist)
{
	truncate_dist = mTruncateDist;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFGeometry::ResetVoxel(unsigned int index)
{
	if ( index>mGeoVolume.size()-1 )
	{
		std::cerr<<"Warning: the index is exceed the size of the volume."<<std::endl;
		return;
	}//end if

	mGeoVolume[ index ].mBlue = 0.0;
	mGeoVolume[ index ].mGreen = 0.0;
	mGeoVolume[ index ].mRed = 0.0;
	mGeoVolume[ index ].mTsdfValue = 0.0;
	mGeoVolume[ index ].mWeight = 0.0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFGeometry::InitVolume()
{
	if ( mGeoVolume.size()!=0 )
		mGeoVolume.resize(0);

	int resX = 0, resY = 0, resZ = 0;
	GetResolution(resX, resY, resZ);
	mGeoVolume.resize( resX * resY * resZ );

	float stepX = mSizeX / mResX;
	float stepY = mSizeY / mResY;
	float stepZ = mSizeZ / mResZ;

	for (int x = 0; x < resX; ++x)
	{
		for (int y = 0; y < resY; ++y)
		{
			for (int z = 0; z < resZ; ++z)
			{
				unsigned int index = x + y * resX + z * resX * resY;

				mGeoVolume[ index ].mBlue = 0.0;
				mGeoVolume[ index ].mGreen = 0.0;
				mGeoVolume[ index ].mRed = 0.0;
				mGeoVolume[ index ].mTsdfValue = -2.0;
				mGeoVolume[ index ].mWeight = 0.0;

				mGeoVolume[ index ].mPos.x = -(x + 0.5) * stepX;
				mGeoVolume[ index ].mPos.y = (y + 0.5) * stepY;
				mGeoVolume[ index ].mPos.z = (z + 0.5) * stepZ;
			}//end inner for
		}//end for
	}//end for
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFGeometry::UpdateVoxel(unsigned int index, cv::Mat &depth_img, cv::Mat &rgb_img, cv::Mat &inv_pose)
{
	/// 1. From world 2 camera.
	cv::Point3f voxelCamPos;
	FromWorld2Camera(mGeoVolume[ index ].mPos, voxelCamPos, inv_pose);

	// Check if the point in front of the camera.
	if ( voxelCamPos.z<=0.0 )
		return;

	// Check if in the frustum. Here set the horizontal and vertical fov to 70 degree.
	if ( voxelCamPos.z<mRangeMin || voxelCamPos.z>mRangeMax )
		return;

	cv::Point3f vProjXZPlane, vProjYZPlane;
	vProjXZPlane.x = voxelCamPos.x;
	vProjXZPlane.y = 0.0;
	vProjXZPlane.z = voxelCamPos.z;
	vProjYZPlane.x = 0.0;
	vProjYZPlane.y = voxelCamPos.y;
	vProjYZPlane.z = voxelCamPos.z;

	float cosThresh = std::cos( 70 * M_PI / ( 180 * 2 ) );

	float cosHorizontal = vProjXZPlane.z / std::sqrt( vProjXZPlane.x * vProjXZPlane.x + vProjXZPlane.y * vProjXZPlane.y + vProjXZPlane.z * vProjXZPlane.z );
	float cosVertical   = vProjYZPlane.z / std::sqrt( vProjYZPlane.x * vProjYZPlane.x + vProjYZPlane.y * vProjYZPlane.y + vProjYZPlane.z * vProjYZPlane.z );

	if ( std::abs( cosHorizontal )<cosThresh || std::abs( cosVertical )<cosThresh )
		return;

	/// 2. Froma camera 2 image, projects into the image.
	cv::Point2i voxelImgPos;
	FromCamera2Image(voxelCamPos, voxelImgPos);

	// Check if project inside the image and valid depth.
	if ( voxelImgPos.x<0 || voxelImgPos.y<0 || voxelImgPos.x>=rgb_img.cols || voxelImgPos.y>=rgb_img.rows )
		return;

	float depthVal = depth_img.at<ushort>(voxelImgPos.y, voxelImgPos.x) / mCamera.mFactor;
	if ( depthVal<mRangeMin || depthVal>mRangeMax )
		return;

	// Get the correspondent camera point.
	cv::Point3f imgPos;
	imgPos.x = voxelImgPos.x;
	imgPos.y = voxelImgPos.y;
	imgPos.z = depthVal * mCamera.mFactor;

	cv::Point3f backCamPos;
	FromImage2Camera(imgPos, backCamPos);
	if ( !backCamPos.x )
		return;

	/// 3. Calculate the signed distance function value.
	float voxelDist = std::sqrt( voxelCamPos.x * voxelCamPos.x + voxelCamPos.y * voxelCamPos.y + voxelCamPos.z * voxelCamPos.z );
	float backDist = std::sqrt( backCamPos.x * backCamPos.x + backCamPos.y * backCamPos.y + backCamPos.z * backCamPos.z );
	float sdfVal = voxelDist - backDist;

	/// 4. Truncate the sdf value.
	if ( sdfVal<-mTruncateDist )
		return;

	float tsdfVal = sdfVal / mTruncateDist;
	if ( sdfVal>0 )
		tsdfVal = tsdfVal<1.0?tsdfVal:1.0;
	else
		tsdfVal = tsdfVal>-1.0?tsdfVal:-1.0;

	/// 4. Update.
	uchar red = rgb_img.at<cv::Vec3b>(voxelImgPos.y, voxelImgPos.x)[0];
	uchar green = rgb_img.at<cv::Vec3b>(voxelImgPos.y, voxelImgPos.x)[1];
	uchar blue = rgb_img.at<cv::Vec3b>(voxelImgPos.y, voxelImgPos.x)[2];

	if ( mGeoVolume[ index ].mWeight<=0.0 )
	{
		mGeoVolume[ index ].mTsdfValue = tsdfVal;
		mGeoVolume[ index ].mWeight = 1;

		mGeoVolume[ index ].mBlue = blue;
		mGeoVolume[ index ].mGreen = green;
		mGeoVolume[ index ].mRed = red;

		return;
	}//end if

	float prevWeight = mGeoVolume[ index ].mWeight;
	float prevTsdfVal = mGeoVolume[ index ].mTsdfValue;

	float prevRed = mGeoVolume[ index ].mRed;
	float prevGreen = mGeoVolume[ index ].mGreen;
	float prevBlue = mGeoVolume[ index ].mBlue;

	float currWeight = prevWeight + 1.0;

	mGeoVolume[ index ].mWeight = currWeight<mMaxWeight?currWeight:mMaxWeight;
	mGeoVolume[ index ].mTsdfValue = (prevTsdfVal * prevWeight + tsdfVal) / currWeight;
	//mGeoVolume[ index ].mTsdfValue = (prevTsdfVal * prevWeight + tsdfVal * currWeight) / (prevWeight + currWeight);
	mGeoVolume[ index ].mRed = (prevRed * prevWeight + red) / currWeight;
	mGeoVolume[ index ].mGreen = (prevGreen * prevWeight + green) / currWeight;
	mGeoVolume[ index ].mBlue = (prevBlue * prevWeight + blue) / currWeight;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFGeometry::CalcAndUpdate(unsigned int index, cv::Mat &depth_img, cv::Mat &rgb_img, cv::Mat &inv_pose)
{
	/// 1. From world 2 camera.
	cv::Point3f voxelCamPos;
	FromWorld2Camera(mGeoVolume[ index ].mPos, voxelCamPos, inv_pose);

	// Check if the point in front of the camera.
	if ( voxelCamPos.z<=0.0 )
		return;

	/// 2. Froma camera 2 image, projects into the image.
	cv::Point2i voxelImgPos;
	FromCamera2Image(voxelCamPos, voxelImgPos);

	// Check if project inside the image and valid depth.
	if ( voxelImgPos.x<0 || voxelImgPos.y<0 || voxelImgPos.x>=rgb_img.cols || voxelImgPos.y>=rgb_img.rows )
		return;

	float depthVal = depth_img.at<ushort>(voxelImgPos.y, voxelImgPos.x) / mCamera.mFactor;
	if ( depthVal<mRangeMin || depthVal>mRangeMax )
		return;

	/// 3. Calculate the signed distance function value.
	//float voxelDist = std::sqrt( voxelCamPos.x * voxelCamPos.x + voxelCamPos.y * voxelCamPos.y + voxelCamPos.z * voxelCamPos.z );
	//float sdfVal = voxelDist - depthVal;
	float sdfVal = voxelCamPos.z - depthVal;

	/// 4. Truncate the sdf value.
	if ( sdfVal<-mTruncateDist )
		return;

	float tsdfVal = sdfVal / mTruncateDist;
	if ( sdfVal>0 )
		tsdfVal = tsdfVal<1.0?tsdfVal:1.0;
	else
		tsdfVal = tsdfVal>-1.0?tsdfVal:-1.0;

	/// 4. Update.
	uchar red = rgb_img.at<cv::Vec3b>(voxelImgPos.y, voxelImgPos.x)[0];
	uchar green = rgb_img.at<cv::Vec3b>(voxelImgPos.y, voxelImgPos.x)[1];
	uchar blue = rgb_img.at<cv::Vec3b>(voxelImgPos.y, voxelImgPos.x)[2];

	if ( mGeoVolume[ index ].mWeight<=0.0 )
	{
		mGeoVolume[ index ].mTsdfValue = tsdfVal;
		mGeoVolume[ index ].mWeight = 1;

		mGeoVolume[ index ].mBlue = blue;
		mGeoVolume[ index ].mGreen = green;
		mGeoVolume[ index ].mRed = red;

		return;
	}//end if

	float prevWeight = mGeoVolume[ index ].mWeight;
	float prevTsdfVal = mGeoVolume[ index ].mTsdfValue;

	float prevRed = mGeoVolume[ index ].mRed;
	float prevGreen = mGeoVolume[ index ].mGreen;
	float prevBlue = mGeoVolume[ index ].mBlue;

	float currWeight = prevWeight + 1.0;

	mGeoVolume[ index ].mWeight = currWeight<mMaxWeight?currWeight:mMaxWeight;
	mGeoVolume[ index ].mTsdfValue = (prevTsdfVal * prevWeight + tsdfVal) / currWeight;
	//mGeoVolume[ index ].mTsdfValue = (prevTsdfVal * prevWeight + tsdfVal * currWeight) / (prevWeight + currWeight);
	mGeoVolume[ index ].mRed = (prevRed * prevWeight + red) / currWeight;
	mGeoVolume[ index ].mGreen = (prevGreen * prevWeight + green) / currWeight;
	mGeoVolume[ index ].mBlue = (prevBlue * prevWeight + blue) / currWeight;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFGeometry::Save()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointXYZRGB point;

	for (int x = 0; x < mResX; ++x)
	{
		for (int y = 0; y < mResY; ++y)
		{
			for (int z = 0; z < mResZ; ++z)
			{
				unsigned int index = x + y * mResX + z * mResX * mResY;
				//if ( mGeoVolume[ index ].mTsdfValue!=-2 )
				//{
				//	std::cout<<"tsdf value: "<<mGeoVolume[ index ].mTsdfValue<<std::endl;
				//	std::cout<<"weight: "<<mGeoVolume[ index ].mWeight<<std::endl;
				//}//end if

				if ( std::abs( mGeoVolume[ index ].mTsdfValue )<mSaveThresh )
				{
					point.x = mGeoVolume[ index ].mPos.x;
					point.y = mGeoVolume[ index ].mPos.y;
					point.z = mGeoVolume[ index ].mPos.z;

					point.b = mGeoVolume[ index ].mRed;
					point.g = mGeoVolume[ index ].mGreen;
					point.r = mGeoVolume[ index ].mBlue;

					cloud->points.push_back( point );
				}//end if
			}//end inner for
		}//end for
	}//end for

	if ( cloud->points.size()<=0 )
	{
		std::cout<<"No point to save."<<std::endl;
		return;
	}//end if

	cloud->width = cloud->points.size();
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->points.resize( cloud->width * cloud->height );

	pcl::io::savePCDFileASCII("data/tsdfGeometryResult.pcd", *cloud);

	/// For debug.
	std::cout<<"cloud size: "<<cloud->points.size()<<std::endl;
	pcl::visualization::CloudViewer viewer("CloudViewer");
	viewer.showCloud(cloud);
	while ( !viewer.wasStopped() )
	{
	}//end while
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFGeometry::SetSaveThresh(float save_thresh)
{
	mSaveThresh = save_thresh;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFGeometry::GetSaveThresh(float &save_thresh)
{
	save_thresh = mSaveThresh;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFGeometry::GetTSDFCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tsdf_cloud)
{
	pcl::PointXYZRGB pt;

	for (int x = 0; x < mResX; ++x)
	{
		for (int y = 0; y < mResY; ++y)
		{
			for (int z = 0; z < mResZ; ++z)
			{
				unsigned int index = x + y * mResX + z * mResX * mResY;

				if ( std::fabs( mGeoVolume[ index ].mTsdfValue )<mSaveThresh )
				{
					pt.x = mGeoVolume[ index ].mPos.x;
					pt.y = mGeoVolume[ index ].mPos.y;
					pt.z = mGeoVolume[ index ].mPos.z;

					pt.r = mGeoVolume[ index ].mBlue;
					pt.g = mGeoVolume[ index ].mGreen;
					pt.b = mGeoVolume[ index ].mRed;

					tsdf_cloud->points.push_back( pt );
				}//end if
			}//end inner for
		}//end for
	}//end for

	tsdf_cloud->width = tsdf_cloud->points.size();
	tsdf_cloud->height = 1;
	tsdf_cloud->is_dense = false;
	tsdf_cloud->points.resize( tsdf_cloud->width * tsdf_cloud->height );
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFGeometry::GetValidVolume(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tsdf_volume)
{
	// Get point cloud.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	GetTSDFCloud(cloud);

	// Get bounding box.
	Eigen::Vector4f minPt, maxPt;
	pcl::getMinMax3D(*cloud, minPt, maxPt);
	Eigen::Vector4f boxSize = maxPt - minPt;

	// Get volume.
	float stepX = mSizeX / mResX;
	float stepY = mSizeY / mResY;
	float stepZ = mSizeZ / mResZ;

	int startIdxX = -minPt.x() / stepX - 0.5;
	int startIdxY = minPt.y() / stepY - 0.5;
	int startIdxZ = minPt.z() / stepZ - 0.5;

	int endIdxX = -maxPt.x() / stepX - 0.5;
	int endIdxY = maxPt.y() / stepY - 0.5;
	int endIdxZ = maxPt.z() / stepZ - 0.5;

	pcl::PointXYZRGB pt;

	for (int x = startIdxX; x <= endIdxX; ++x)
	{
		for (int y = startIdxY; y <= endIdxY; ++y)
		{
			for (int z = startIdxZ; z <= endIdxZ; ++z)
			{
				unsigned int index = x + y * mResX + z * mResX * mResY;

				pt.x = mGeoVolume[ index ].mPos.x;
				pt.y = mGeoVolume[ index ].mPos.y;
				pt.z = mGeoVolume[ index ].mPos.z;

				pt.r = mGeoVolume[ index ].mBlue;
				pt.g = mGeoVolume[ index ].mGreen;
				pt.b = mGeoVolume[ index ].mRed;

				tsdf_volume->points.push_back( pt );
			}//end inner for
		}//end for
	}//end for

	tsdf_volume->width = tsdf_volume->points.size();
	tsdf_volume->height = 1;
	tsdf_volume->is_dense = false;
	tsdf_volume->points.resize( tsdf_volume->width * tsdf_volume->height );
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFGeometry::GetVoxel(int index, GeometryVoxel &voxel)
{
	if ( index<0 || index>=mGeoVolume.size() )
		return;

	voxel = mGeoVolume[ index ];
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float TSDFGeometry::GetWeight(unsigned int index)
{
	if ( index<0 || index>=mGeoVolume.size() )
		return -1.0;

	return mGeoVolume[ index ].mWeight;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float TSDFGeometry::GetTsdfValue(unsigned int index)
{
	if ( index<0 || index>=mGeoVolume.size() )
		return -1.0;

	return mGeoVolume[ index ].mTsdfValue;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFGeometry::GetRGB(unsigned int index, uchar &r, uchar &g, uchar &b)
{
	r = static_cast<uchar>( mGeoVolume[index].mRed );
	g = static_cast<uchar>( mGeoVolume[index].mGreen );
	b = static_cast<uchar>( mGeoVolume[index].mBlue );
}

///////////////////////////////////////////////////////////////////////////////////////////////////

