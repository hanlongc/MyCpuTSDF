#include "tsdfTexture.h"

///////////////////////////////////////////////////////////////////////////////////////////////////

TSDFTexture::TSDFTexture()
	: TSDFBase()
	, mTheta( 0.0 )
{

}

///////////////////////////////////////////////////////////////////////////////////////////////////

TSDFTexture::~TSDFTexture()
{

}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFTexture::UpdateVoxel(Voxel &voxel, cv::Mat &inv_pose, cv::Mat &depth_img, cv::Mat &rgb_img)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFTexture::UpdateVoxel(std::shared_ptr<Voxel> voxel, cv::Mat &inv_pose, cv::Mat &depth_img, cv::Mat &rgb_img)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFTexture::UpdateVolume(cv::Mat &depth_img, cv::Mat &rgb_img, cv::Mat &inv_pose)
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

				//UpdateVoxel(index, depth_img, rgb_img, inv_pose, pos);
				CalcAndUpdate(index, depth_img, rgb_img, inv_pose, pos);
			}//end inner for
		}//end for
	}//end for
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFTexture::UpdateVoxel(unsigned int index, cv::Mat &depth_img, cv::Mat &rgb_img, cv::Mat &inv_pose, cv::Point3f &voxel_pos)
{
	/// 0. From world to camera.
	cv::Point3f voxelCamPos;
	FromWorld2Camera(voxel_pos, voxelCamPos, inv_pose);

	// Check if the point is valid.
	if ( voxelCamPos.z<=0.0 )
		return;

	// Check if in the camera frustum.
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

	/// 1. From camera to image.
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

	/// 2. Check if the distance between voxel point and back point is smaller than a threshold.
	float xx = (voxelCamPos.x - backCamPos.x) * (voxelCamPos.x - backCamPos.x);
	float yy = (voxelCamPos.y - backCamPos.y) * (voxelCamPos.y - backCamPos.y);
	float zz = (voxelCamPos.z - backCamPos.z) * (voxelCamPos.z - backCamPos.z);
	float dist = std::sqrt( xx + yy + zz );
	if ( dist>mTheta )
		return;

	/// 3. Caculate the weight.
	int count = 0;
	float currWeight;
	while( 1 )
	{
		count++;
		
		if ( (voxelImgPos.x + count)>=depth_img.cols || (voxelImgPos.y + count)>=depth_img.rows )
			return;

		ushort depth1 = depth_img.at<ushort>(voxelImgPos.y, (voxelImgPos.x + count));
		ushort depth2 = depth_img.at<ushort>((voxelImgPos.y + count), voxelImgPos.x);
		if ( depth1<=0 || depth2<=0 )
			continue;

		cv::Point3f imgPos1, imgPos2;
		cv::Point3f backCamPos1, backCamPos2;

		imgPos1.x = imgPos.x + count;
		imgPos1.y = imgPos.y;
		imgPos1.z = depth1;
		imgPos2.x = imgPos.x;
		imgPos2.y = imgPos.y + count;
		imgPos2.z = depth2;

		FromImage2Camera(imgPos1, backCamPos1);
		FromImage2Camera(imgPos2, backCamPos2);

		cv::Point3f vector1, vector2;
		vector1.x = backCamPos1.x - backCamPos.x;
		vector1.y = backCamPos1.y - backCamPos.y;
		vector1.z = backCamPos1.z - backCamPos.z;
		vector2.x = backCamPos2.x - backCamPos.x;
		vector2.y = backCamPos2.y - backCamPos.y;
		vector2.z = backCamPos2.z - backCamPos.z;

		cv::Point3f pointNormal;
		CrossProduct(vector1, vector2, pointNormal);

		cv::Point3f normalizedPointN, normalizedViewDire;
		Normalize(pointNormal, normalizedPointN);
		Normalize(backCamPos, normalizedViewDire);

		currWeight = DotProduct(normalizedPointN, normalizedViewDire);

		break;
	}//end while

	float prevW = mColorVolume[ index ].mWeight;
	
	if ( currWeight < (prevW * 0.8) )
		return;

	uchar prevR, prevG, prevB, currR, currG, currB;

	currB = rgb_img.at<cv::Vec3b>(voxelImgPos.y, voxelImgPos.x)[0];
	currG = rgb_img.at<cv::Vec3b>(voxelImgPos.y, voxelImgPos.x)[1];
	currR = rgb_img.at<cv::Vec3b>(voxelImgPos.y, voxelImgPos.x)[2];

	prevR = mColorVolume[ index ].mRed;
	prevG = mColorVolume[ index ].mGreen;
	prevB = mColorVolume[ index ].mBlue;
	//prevW = mColorVolume[ index ].mWeight;

	float divideW = prevW + currWeight;

	mColorVolume[ index ].mRed    = static_cast<unsigned int>( (prevR * prevW + currWeight * currR) / divideW );
	mColorVolume[ index ].mGreen  = static_cast<unsigned int>( (prevG * prevW + currWeight * currG) / divideW );
	mColorVolume[ index ].mBlue   = static_cast<unsigned int>( (prevR * prevW + currWeight * currB) / divideW );
	mColorVolume[ index ].mWeight = prevW + currWeight * (1 - prevW);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFTexture::CalcAndUpdate(unsigned int index, cv::Mat &depth_img, cv::Mat &rgb_img, cv::Mat &inv_pose, cv::Point3f &voxel_pos)
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
	float xx = (voxelCamPos.x - backCamPos.x) * (voxelCamPos.x - backCamPos.x);
	float yy = (voxelCamPos.y - backCamPos.y) * (voxelCamPos.y - backCamPos.y);
	float zz = (voxelCamPos.z - backCamPos.z) * (voxelCamPos.z - backCamPos.z);
	float dist = std::sqrt( xx + yy + zz );
	if ( dist>mTheta )
		return;
	
	/// 4. Update.
	uchar blue = rgb_img.at<cv::Vec3b>(voxelImgPos.y, voxelImgPos.x)[0];
	uchar green = rgb_img.at<cv::Vec3b>(voxelImgPos.y, voxelImgPos.x)[1];
	uchar red = rgb_img.at<cv::Vec3b>(voxelImgPos.y, voxelImgPos.x)[2];

	if ( mColorVolume[ index ].mWeight<=0.0 )
	{
		mColorVolume[ index ].mWeight = 1;

		mColorVolume[ index ].mBlue = blue;
		mColorVolume[ index ].mGreen = green;
		mColorVolume[ index ].mRed = red;

		return;
	}//end if

	float prevWeight = mColorVolume[ index ].mWeight;

	uchar prevRed = mColorVolume[ index ].mRed;
	uchar prevGreen = mColorVolume[ index ].mGreen;
	uchar prevBlue = mColorVolume[ index ].mBlue;

	float currWeight = prevWeight + 1.0;

	mColorVolume[ index ].mWeight = currWeight<mMaxWeight?currWeight:mMaxWeight;

	uchar newRed   = static_cast<unsigned int>( (prevRed * prevWeight + red) / currWeight );
	uchar newGreen = static_cast<unsigned int>( (prevGreen * prevWeight + green) / currWeight );
	uchar newBlue  = static_cast<unsigned int>( (prevBlue * prevWeight + blue) / currWeight );

	mColorVolume[ index ].mRed = newRed>255?255:newRed;
	mColorVolume[ index ].mGreen = newGreen>255?255:newGreen;
	mColorVolume[ index ].mBlue = newBlue>255?255:newBlue;
	/*mColorVolume[ index ].mRed = static_cast<unsigned int>( (prevRed * prevWeight + red) / currWeight );
	mColorVolume[ index ].mGreen = static_cast<unsigned int>( (prevGreen * prevWeight + green) / currWeight );
	mColorVolume[ index ].mBlue = static_cast<unsigned int>( (prevBlue * prevWeight + blue) / currWeight );*/
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFTexture::SetTheta(float theta)
{
	mTheta = theta;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFTexture::GetTheta(float &theta)
{
	theta = mTheta;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFTexture::ResetVoxel(int index)
{
	if (index<0 || index>=mColorVolume.size())
		return;

	mColorVolume[index].mBlue = 0;
	mColorVolume[index].mGreen = 0;
	mColorVolume[index].mRed = 0;

	mColorVolume[index].mWeight = 0.0;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFTexture::InitVolume()
{
	if ( mColorVolume.size()!=0 )
		mColorVolume.resize( 0 );

	mColorVolume.resize( mResX * mResY * mResZ );

	for (int x = 0; x < mResX; ++x)
	{
		for (int y = 0; y < mResY; ++y)
		{
			for (int z = 0; z < mResZ; ++z)
			{
				unsigned int index = x + y * mResX + z * mResX * mResY;

				ResetVoxel(index);
			}//end for
		}//end for
	}//end for
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFTexture::Save()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TSDFTexture::GetRGB(unsigned int index, uchar &r, uchar &g, uchar &b)
{
	if ( index<0 || index>=mColorVolume.size() )
		return;

	r = mColorVolume[ index ].mRed;
	g = mColorVolume[ index ].mGreen;
	b = mColorVolume[ index ].mBlue;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

