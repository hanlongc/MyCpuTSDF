#include "tsdfVolumetricFusion.h"

///////////////////////////////////////////////////////////////////////////////////////////////////

TSDFVolumetricFusion::TSDFVolumetricFusion()
	: TSDFBase()
	, mTruncateDist( 0.0 )
	, mSaveThresh( 0.0 )
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

TSDFVolumetricFusion::~TSDFVolumetricFusion()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFVolumetricFusion::UpdateVolume(cv::Mat &depth_img, cv::Mat &rgb_img, cv::Mat &inv_pose)
{
	float stepX = mSizeX / mResX;
	float stepY = mSizeY / mResY;
	float stepZ = mSizeZ / mResZ;

	for (int x = 0; x < mResX; ++x)
	{
		for (int y = 0; y < mResY; ++y)
		{
			for (int z = 0; z < mResZ; ++z)
			{
				unsigned int index = x + y * mResX + z * mResX * mResY;

				cv::Point3f pos;
				pos.x = -(x + 0.5) * stepX;
				pos.y =  (y + 0.5) * stepY;
				pos.z =  (z + 0.5) * stepZ;

				UpdateVoxel(index, depth_img, rgb_img, inv_pose, pos);
				//CalcAndUpdate(index, depth_img, rgb_img, inv_pose, pos);
			}//end inner for
		}//end for
	}//end for
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFVolumetricFusion::SetTruncateDist(float truncate_dist)
{
	mTruncateDist = truncate_dist;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFVolumetricFusion::GetTruncateDist(float &truncate_dist)
{
	truncate_dist = mTruncateDist;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFVolumetricFusion::ResetVoxel(unsigned int index)
{
	if ( index>mGeoVolume.size()-1 )
	{
		std::cerr<<"Warning: the index is exceed the size of the volume."<<std::endl;
		return;
	}//end if

	mGeoVolume[ index ].mBlue = 0;
	mGeoVolume[ index ].mGreen = 0;
	mGeoVolume[ index ].mRed = 0;
	mGeoVolume[ index ].mTsdfValue = 0.0;
	mGeoVolume[ index ].mWeight = 0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFVolumetricFusion::InitVolume()
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

				mGeoVolume[ index ].mBlue = 0;
				mGeoVolume[ index ].mGreen = 0;
				mGeoVolume[ index ].mRed = 0;
				mGeoVolume[ index ].mTsdfValue = -2.0;
				mGeoVolume[ index ].mWeight = 0;
			}//end inner for
		}//end for
	}//end for
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFVolumetricFusion::UpdateVoxel(unsigned int index, cv::Mat &depth_img, cv::Mat &rgb_img, cv::Mat &inv_pose, cv::Point3f &voxel_pos)
{
	/// 1. From world 2 camera.
	cv::Point3f voxelCamPos;
	FromWorld2Camera(voxel_pos, voxelCamPos, inv_pose);

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

	if ( mGeoVolume[ index ].mWeight<=0 )
	{
		mGeoVolume[ index ].mTsdfValue = tsdfVal;
		mGeoVolume[ index ].mWeight = 1;

		mGeoVolume[ index ].mBlue = blue;
		mGeoVolume[ index ].mGreen = green;
		mGeoVolume[ index ].mRed = red;

		return;
	}//end if

	uchar prevWeight = mGeoVolume[ index ].mWeight;
	float prevTsdfVal = mGeoVolume[ index ].mTsdfValue;

	uchar prevRed = mGeoVolume[ index ].mRed;
	uchar prevGreen = mGeoVolume[ index ].mGreen;
	uchar prevBlue = mGeoVolume[ index ].mBlue;

	uchar currWeight = (int)(prevWeight) + 1;

	mGeoVolume[ index ].mWeight = ((int)(currWeight)<mMaxWeight)?currWeight:(int)(mMaxWeight+0.5);
	mGeoVolume[ index ].mTsdfValue = (prevTsdfVal * (int)(prevWeight) + tsdfVal) / (int)(currWeight);
	//mGeoVolume[ index ].mTsdfValue = (prevTsdfVal * prevWeight + tsdfVal * (int)(currWeight)) / (int)(prevWeight + currWeight);
	mGeoVolume[ index ].mRed = ((int)(prevRed) * (int)(prevWeight) + (int)(red)) / (int)(currWeight);
	mGeoVolume[ index ].mGreen = ((int)(prevGreen) * (int)(prevWeight) + (int)(green)) / (int)(currWeight);
	mGeoVolume[ index ].mBlue = ((int)(prevBlue) * (int)(prevWeight) + (int)(blue)) / (int)(currWeight);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFVolumetricFusion::CalcAndUpdate(unsigned int index, cv::Mat &depth_img, cv::Mat &rgb_img, cv::Mat &inv_pose, cv::Point3f &voxel_pos)
{
	/// 1. From world 2 camera.
	cv::Point3f voxelCamPos;
	FromWorld2Camera(voxel_pos, voxelCamPos, inv_pose);

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

	uchar prevWeight = mGeoVolume[ index ].mWeight;
	float prevTsdfVal = mGeoVolume[ index ].mTsdfValue;

	uchar prevRed = mGeoVolume[ index ].mRed;
	uchar prevGreen = mGeoVolume[ index ].mGreen;
	uchar prevBlue = mGeoVolume[ index ].mBlue;

	uchar currWeight = prevWeight + 1;

	mGeoVolume[ index ].mWeight = (int)(currWeight)<mMaxWeight?currWeight:(int)(mMaxWeight+0.5);
	mGeoVolume[ index ].mTsdfValue = (prevTsdfVal * prevWeight + tsdfVal) / currWeight;
	//mGeoVolume[ index ].mTsdfValue = (prevTsdfVal * prevWeight + tsdfVal * currWeight) / (prevWeight + currWeight);
	mGeoVolume[ index ].mRed = (prevRed * prevWeight + red) / currWeight;
	mGeoVolume[ index ].mGreen = (prevGreen * prevWeight + green) / currWeight;
	mGeoVolume[ index ].mBlue = (prevBlue * prevWeight + blue) / currWeight;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFVolumetricFusion::Save()
{
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::PointXYZRGB point;

	float stepX = mSizeX / mResX;
	float stepY = mSizeY / mResY;
	float stepZ = mSizeZ / mResZ;

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
					point.x = -(x + 0.5) * stepX;
					point.y =  (y + 0.5) * stepY;
					point.z =  (z + 0.5) * stepZ;

					point.b = (int)mGeoVolume[ index ].mRed;
					point.g = (int)mGeoVolume[ index ].mGreen;
					point.r = (int)mGeoVolume[ index ].mBlue;

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

	pcl::io::savePCDFileASCII("data/tsdfVolumetricFusionResult.pcd", *cloud);

	/// For debug.
	std::cout<<"cloud size: "<<cloud->points.size()<<std::endl;
	pcl::visualization::CloudViewer viewer("CloudViewer");
	viewer.showCloud(cloud);
	while ( !viewer.wasStopped() )
	{
	}//end while
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFVolumetricFusion::SaveVolumeData()
{
	std::ofstream outfile;
	outfile.open("data/tsdfVolumetricFusionResult.vol");
	if ( !outfile.is_open() )
	{
		std::cerr<<"Warning: Cannot open the file <data/tsdfVolumetricFusionResult.vol> to write."<<std::endl;
		return;
	}//end if

	outfile<<"TSDFVolumetricFusion parameters"<<std::endl;
	outfile<<"size_x size_y size_z res_x res_y res_z"<<std::endl;
	outfile<<mSizeX<<" "<<mSizeY<<" "<<mSizeZ<<" "<<mResX<<" "<<mResY<<" "<<mResZ<<std::endl;

	outfile<<"TSDFVolumetricFusion result"<<std::endl;
	outfile<<"voxel_x voxel_y voxel_z voxel_r voxel_g voxel_b voxel_tsdf_value voxel_weight"<<std::endl;

	float stepX = mSizeX / mResX;
	float stepY = mSizeY / mResY;
	float stepZ = mSizeZ / mResZ;

	for (unsigned int x = 0; x < mResX; ++x)
	{
		for (unsigned int y = 0; y < mResY; ++y)
		{
			for (unsigned int z = 0; z < mResZ; ++z)
			{
				unsigned int index = x + y * mResX + z * mResX * mResY;

				if ( std::abs( mGeoVolume[ index ].mTsdfValue )<mSaveThresh )
				{
					outfile<<(-(x + 0.5) * stepX)<<" "<<((y + 0.5) * stepY)<<" "<<((z + 0.5) * stepZ)<<" ";
					outfile<<(int)mGeoVolume[ index ].mBlue<<" "<<(int)mGeoVolume[ index ].mGreen<<" "<<(int)mGeoVolume[ index ].mRed<<" ";
					outfile<<mGeoVolume[ index ].mTsdfValue<<" "<<mGeoVolume[ index ].mWeight<<std::endl;
				}//end if
			}//end inner for
		}//end for
	}//end for

	outfile.close();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFVolumetricFusion::SetSaveThresh(float save_thresh)
{
	mSaveThresh = save_thresh;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFVolumetricFusion::GetSaveThresh(float &save_thresh)
{
	save_thresh = mSaveThresh;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFVolumetricFusion::GetTSDFCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tsdf_cloud)
{
	pcl::PointXYZRGB pt;

	float stepX = mSizeX / mResX;
	float stepY = mSizeY / mResY;
	float stepZ = mSizeZ / mResZ;

	for (int x = 0; x < mResX; ++x)
	{
		for (int y = 0; y < mResY; ++y)
		{
			for (int z = 0; z < mResZ; ++z)
			{
				unsigned int index = x + y * mResX + z * mResX * mResY;

				if ( std::fabs( mGeoVolume[ index ].mTsdfValue )<mSaveThresh )
				{
					pt.x = -(x + 0.5) * stepX;
					pt.y =  (y + 0.5) * stepY;
					pt.z =  (z + 0.5) * stepZ;

					pt.r = (int)mGeoVolume[ index ].mBlue;
					pt.g = (int)mGeoVolume[ index ].mGreen;
					pt.b = (int)mGeoVolume[ index ].mRed;

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

void TSDFVolumetricFusion::GetValidVolume(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tsdf_volume)
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

				pt.x = -(x + 0.5) * stepX;
				pt.y =  (y + 0.5) * stepY;
				pt.z =  (z + 0.5) * stepZ;

				pt.r = (int)mGeoVolume[ index ].mBlue;
				pt.g = (int)mGeoVolume[ index ].mGreen;
				pt.b = (int)mGeoVolume[ index ].mRed;

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

void TSDFVolumetricFusion::GetVoxel(int index, SmallGeometryVoxel &voxel)
{
	if ( index<0 || index>=mGeoVolume.size() )
		return;

	voxel = mGeoVolume[ index ];
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float TSDFVolumetricFusion::GetWeight(unsigned int index)
{
	if ( index<0 || index>=mGeoVolume.size() )
		return -1.0;

	return (float)((int)mGeoVolume[ index ].mWeight + 0.0000001);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

float TSDFVolumetricFusion::GetTsdfValue(unsigned int index)
{
	if ( index<0 || index>=mGeoVolume.size() )
		return -1.0;

	return mGeoVolume[ index ].mTsdfValue;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFVolumetricFusion::GetRGB(unsigned int index, uchar &r, uchar &g, uchar &b)
{
	r = static_cast<uchar>( mGeoVolume[index].mRed );
	g = static_cast<uchar>( mGeoVolume[index].mGreen );
	b = static_cast<uchar>( mGeoVolume[index].mBlue );
}

///////////////////////////////////////////////////////////////////////////////////////////////////

