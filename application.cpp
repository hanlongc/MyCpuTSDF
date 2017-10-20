#include "application.h"

///////////////////////////////////////////////////////////////////////////////////////////////////

Application::Application(int argc, char** argv)
	: mArgc( argc )
	, mArgv( argv )
{
	
}

///////////////////////////////////////////////////////////////////////////////////////////////////

Application::~Application()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Application::Exec()
{
	// Use default data and default settings.
	if (mArgc == 1)
	{
		std::cout << std::endl << "Using default setting." << std::endl << std::endl;

		/// 0. Init default setting.
		InitDefaultSetting();

		/// 1. Get default data.
		std::cout << "Getting data......" << std::endl;
		std::vector<cv::Mat> depthImgs, rgbImgs;
		std::vector<TUMPose> tumPoses;
		GetDefaultData(depthImgs, rgbImgs, tumPoses);
		std::cout << "Get data, done." << std::endl;

		// Check data.
		if (depthImgs.empty() || depthImgs.size() != rgbImgs.size() || rgbImgs.size() != tumPoses.size())
		{
			std::cerr << "Error: no data or invalid data." << std::endl;
			return;
		}//end if

		/// 2. For each frame, do volumetric fusion.
#ifdef _WIN32
		SYSTEMTIME sysPrev, sysLater;
#else
		struct timeval prevTime, laterTime;
#endif
		for (size_t i = 0; i < depthImgs.size(); ++i)
		{
			std::cout << std::endl << "Processing frame: " << i + 1 << std::endl;
#ifdef _WIN32
			GetLocalTime(&sysPrev);
#else
			gettimeofday(&prevTime, NULL);
#endif
			/// 3. Calculate camera pose.
			cv::Mat camPose;
			FromQuanternion2Matrix(tumPoses[i], camPose);

			// Check data.
			if (camPose.empty())
			{
				std::cout << "Warning: pose is empty of frame " << i + 1 << std::endl;
				continue;
			}//end if

			cv::Mat invCamPose = camPose.inv();

			if ( invCamPose.empty() )
			{
				std::cout<< "Warning: inv pose is empty in the frame "<< i+ 1 <<std::endl;
				continue;
			}//end if

			/// 4. Do volumetric fusion.
			mTsdfPtr->UpdateVolume(depthImgs[i], rgbImgs[i], invCamPose);
			
#ifdef _WIN32
			GetLocalTime(&sysLater);
#else
			gettimeofday(&laterTime, NULL);
#endif
			std::cout << "Process frame " << i + 1 << " done." << std::endl;
#ifdef _WIN32
			std::cout << "Time cost: " << TimeCost(sysPrev, sysLater) << " ms." << std::endl;
#else
			double timeCost = (laterTime.tv_sec - prevTime.tv_sec) * 1000 + (laterTime.tv_usec - prevTime.tv_usec) / 1000;
			std::cout << "Time cost: " << timeCost << " ms." << std::endl;
#endif
		}//end for

		/// 5. Save the tsdf result.
		if ( mSaveTsdf.compare("saveTsdf")==0 )
		{
			std::cout<<std::endl<<"Saving the tsdf point cloud......"<<std::endl;
			mTsdfPtr->Save();
			mTsdfPtr->SaveVolumeData();
			std::cout<<"Save the tsdf point cloud, done."<<std::endl;
		}//end if

		if ( mVoxelType.compare("geometry")!=0 && mVoxelType.compare("volumetric")!=0 )
			return;

		/// 6. Meshing.
		if ( mToMesh.compare("m")==0 )
		{
			std::cout<<std::endl<<"Meshing......"<<std::endl;

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			mTsdfPtr->GetTSDFCloud(cloud);

			pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
			tree->setInputCloud( cloud );

			pcl::PolygonMesh mesh;
			float isoLevel = 0.50;

			Eigen::Vector4f minPt, maxPt;
			pcl::getMinMax3D(*cloud, minPt, maxPt);
			Eigen::Vector4f boxSize = maxPt - minPt;

			float resXF = boxSize.x() * mResX / mSizeX;
			float resYF = boxSize.y() * mResY / mSizeY;
			float resZF = boxSize.z() * mResZ / mSizeZ;

			int resX = (int)resXF;
			int resY = (int)resYF;
			int resZ = (int)resZF;

			if ( resXF - resX > 0.5 )
				resX++;
			if ( resYF - resY > 0.5 )
				resY++;
			if ( resZF - resZ > 0.5 )
				resZ++;

			MarchingCube mc;

			mc.setInputCloud( cloud );
			mc.SetMinWeight( 2.0 );
			mc.SetTsdfPtr( mTsdfPtr );
			mc.setGridResolution(resX, resY, resZ);
			mc.setIsoLevel( isoLevel );
			mc.setSearchMethod( tree );

			mc.reconstruct( mesh );

			std::cout<<"Meshing, done."<<std::endl;
			std::cout<<std::endl<<"Saving mesh result......"<<std::endl;

			pcl::io::savePLYFile("myMesh.ply", mesh);

			std::cout<<"Save mesh, done."<<std::endl;
		}//end if
	}//end if
	else if ( mArgc==2 )
	{
		std::cout << std::endl << "Using color volume default setting." << std::endl << std::endl;

		/// 0. Init default setting.
		InitDefaultSetting();

		/// 1. Get default data.
		std::cout << "Getting data......" << std::endl;
		std::vector<cv::Mat> depthImgs, rgbImgs;
		std::vector<TUMPose> tumPoses;
		GetDefaultData(depthImgs, rgbImgs, tumPoses);
		std::cout << "Get data, done." << std::endl;

		// Check data.
		if (depthImgs.empty() || depthImgs.size() != rgbImgs.size() || rgbImgs.size() != tumPoses.size())
		{
			std::cerr << "Error: no data or invalid data." << std::endl;
			return;
		}//end if

		/// 2. For each frame, do volumetric fusion.
#ifdef _WIN32
		SYSTEMTIME sysPrev, sysLater;
#else
		struct timeval prevTime, laterTime;
#endif
		for (size_t i = 0; i < depthImgs.size(); ++i)
		{
			std::cout << std::endl << "Processing frame: " << i + 1 << std::endl;
#ifdef _WIN32
			GetLocalTime(&sysPrev);
#else
			gettimeofday(&prevTime, NULL);
#endif
			/// 3. Calculate camera pose.
			cv::Mat camPose;
			FromQuanternion2Matrix(tumPoses[i], camPose);

			// Check data.
			if (camPose.empty())
			{
				std::cout << "Warning: pose is empty of frame " << i + 1 << std::endl;
				continue;
			}//end if

			cv::Mat invCamPose = camPose.inv();

			if ( invCamPose.empty() )
			{
				std::cout<< "Warning: inv pose is empty in the frame "<< i+ 1 <<std::endl;
				continue;
			}//end if

			/// 4. Do volumetric fusion.
			mTsdfPtr->UpdateVolume(depthImgs[i], rgbImgs[i], invCamPose);
			
#ifdef _WIN32
			GetLocalTime(&sysLater);
#else
			gettimeofday(&laterTime, NULL);
#endif
			std::cout << "Process frame " << i + 1 << " done." << std::endl;
#ifdef _WIN32
			std::cout << "Time cost: " << TimeCost(sysPrev, sysLater) << " ms." << std::endl;
#else
			double timeCost = (laterTime.tv_sec - prevTime.tv_sec) * 1000 + (laterTime.tv_usec - prevTime.tv_usec) / 1000;
			std::cout << "Time cost: " << timeCost << " ms." << std::endl;
#endif
		}//end for
	}//end else if
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Application::Run()
{
	// Use default data and default settings.
	// Here I take the first frame camera pose as the world coordinate.
	if (mArgc == 1)
	{
		std::cout << std::endl << "Using default setting." << std::endl << std::endl;

		/// 0. Init default setting.
		InitDefaultSetting();

		/// 1. Get default data.
		std::cout << "Getting data......" << std::endl;
		std::vector<cv::Mat> depthImgs, rgbImgs;
		std::vector<TUMPose> tumPoses;
		GetDefaultData(depthImgs, rgbImgs, tumPoses);
		std::cout << "Get data, done." << std::endl;

		// Check data.
		if (depthImgs.empty() || depthImgs.size() != rgbImgs.size() || rgbImgs.size() != tumPoses.size())
		{
			std::cerr << "Error: no data or invalid data." << std::endl;
			return;
		}//end if

		/// 2. For each frame, do volumetric fusion.
#ifdef _WIN32
		SYSTEMTIME sysPrev, sysLater;
#else
		struct timeval prevTime, laterTime;
#endif

		// The camera pose of the first frame.
		cv::Mat firstCamPose;
		firstCamPose.create(4, 4, CV_32FC1);
		firstCamPose.setTo( 0 );

		for (size_t i = 0; i < depthImgs.size(); ++i)
		{
			std::cout << std::endl << "Processing frame: " << i + 1 << std::endl;
#ifdef _WIN32
			GetLocalTime(&sysPrev);
#else
			gettimeofday(&prevTime, NULL);
#endif
			/// 3. Calculate camera pose.
			cv::Mat camPose;
			FromQuanternion2Matrix(tumPoses[i], camPose);

			if ( i==0 )
			{
				camPose.copyTo( firstCamPose );
			}//end if

			// Check data.
			if (camPose.empty())
			{
				std::cout << "Warning: pose is empty of frame " << i + 1 << std::endl;
				continue;
			}//end if

			cv::Mat invCamPose = camPose.inv();

			if ( i==0 )
			{
				invCamPose.setTo( 0 );

				invCamPose.at<float>(0, 0) = 1.0;
				invCamPose.at<float>(1, 1) = 1.0;
				invCamPose.at<float>(2, 2) = 1.0;
				invCamPose.at<float>(3, 3) = 1.0;
			}//end if
			else
			{
				cv::Mat tmpPose;
				tmpPose.create(4, 4, CV_32FC1);
				tmpPose.setTo( 0 );

				invCamPose.copyTo( tmpPose );

				MatrixXMatrix(tmpPose, firstCamPose, invCamPose);
			}//end else

			if ( invCamPose.empty() )
			{
				std::cout<< "Warning: inv pose is empty in the frame "<< i+ 1 <<std::endl;
				continue;
			}//end if

			/// 4. Do volumetric fusion.
			mTsdfPtr->UpdateVolume(depthImgs[i], rgbImgs[i], invCamPose);
			
#ifdef _WIN32
			GetLocalTime(&sysLater);
#else
			gettimeofday(&laterTime, NULL);
#endif
			std::cout << "Process frame " << i + 1 << " done." << std::endl;
#ifdef _WIN32
			std::cout << "Time cost: " << TimeCost(sysPrev, sysLater) << " ms." << std::endl;
#else
			double timeCost = (laterTime.tv_sec - prevTime.tv_sec) * 1000 + (laterTime.tv_usec - prevTime.tv_usec) / 1000;
			std::cout << "Time cost: " << timeCost << " ms." << std::endl;
#endif
		}//end for

		/// 5. Save the tsdf result.
		if ( mSaveTsdf.compare("saveTsdf")==0 )
		{
			std::cout<<std::endl<<"Saving the tsdf point cloud......"<<std::endl;
			mTsdfPtr->Save();
			mTsdfPtr->SaveVolumeData();
			std::cout<<"Save the tsdf point cloud, done."<<std::endl;
		}//end if

		/// 6. Meshing.
		if ( mToMesh.compare("m")==0 )
		{
			std::cout<<std::endl<<"Meshing......"<<std::endl;

			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
			mTsdfPtr->GetTSDFCloud(cloud);

			pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
			tree->setInputCloud( cloud );

			pcl::PolygonMesh mesh;
			float isoLevel = 0.50;

			Eigen::Vector4f minPt, maxPt;
			pcl::getMinMax3D(*cloud, minPt, maxPt);
			Eigen::Vector4f boxSize = maxPt - minPt;

			int resX = boxSize.x() * mResX / mSizeX;
			int resY = boxSize.y() * mResY / mSizeY;
			int resZ = boxSize.z() * mResZ / mSizeZ;

			MarchingCube mc;

			mc.setInputCloud( cloud );
			mc.SetMinWeight( 2.0 );
			mc.SetTsdfPtr( mTsdfPtr );
			mc.setGridResolution(resX, resY, resZ);
			mc.setIsoLevel( isoLevel );
			mc.setSearchMethod( tree );

			mc.reconstruct( mesh );

			std::cout<<"Meshing, done."<<std::endl;
			std::cout<<std::endl<<"Saving mesh result......"<<std::endl;

			pcl::io::savePLYFile("myMesh.ply", mesh);

			std::cout<<"Save mesh, done."<<std::endl;
		}//end if
	}//end if
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Application::GetDefaultData(std::vector<cv::Mat> &depth_imgs, std::vector<cv::Mat> &rgb_imgs, std::vector<TUMPose> &tum_poses)
{
	std::ifstream infile;
	infile.open("data/association.txt");
	if (!infile.is_open())
	{
		std::cout << "Cannot open file <data/association.txt>." << std::endl;
		return;
	}//end if

	char temp[256];
	infile.getline(temp, 256);

	Association ass;
	std::vector<Association> asses;
	while (infile >> ass.mRGBTimestamp && infile >> ass.mRGBImgName && infile >> ass.mDepthTimestamp && infile >> ass.mDepthImgName)
	{
		asses.push_back(ass);
		if (asses.size() >= mFrameNum)
			break;
	}//end while
	infile.close();

	infile.open("data/groundtruth.txt");
	if (!infile.is_open())
	{
		std::cerr << "Cannot open the file <data/groundtruth.txt>." << std::endl;
		return;
	}//end if

	for (int i = 0; i<4; ++i)
		infile.getline(temp, 256);

	TUMPose tumPose;
	//std::vector<TUMPose> tumPoses;
	//tumPoses.resize( asses.size() );
	tum_poses.resize(asses.size());
	int poseCount = 0;

	std::vector<int> status;
	for (int i = 0; i<asses.size(); ++i)
		status.push_back(0);

	while (infile >> tumPose.mTS && infile >> tumPose.mTX && infile >> tumPose.mTY &&
		infile >> tumPose.mTZ && infile >> tumPose.mQX && infile >> tumPose.mQY && infile >> tumPose.mQZ && infile >> tumPose.mQW)
	{
		for (size_t i = 0; i<asses.size(); ++i)
		{
			double tsDiff = std::abs(tumPose.mTS - asses[i].mDepthTimestamp);
			if (tsDiff<0.01 && status[i] == 0)
			{
				status[i] = 1;
				poseCount++;
				//tumPoses[i] = tumPose;
				tum_poses[i] = tumPose;
				break;
			}//end if
		}//end for

		if (poseCount >= asses.size())
			break;
	}//end while
	infile.close();

	for (size_t i = 0; i<asses.size(); ++i)
	{
		std::string depthImgPath = "data/" + asses[i].mDepthImgName;
		std::string rgbImgPath = "data/" + asses[i].mRGBImgName;

		cv::Mat depthImg = cv::imread(depthImgPath, -1);
		cv::Mat rgbImg = cv::imread(rgbImgPath, 1);

		if (!depthImg.data || !rgbImg.data)
			std::cout << "Error: no input image." << std::endl;

		depth_imgs.push_back(depthImg);
		rgb_imgs.push_back(rgbImg);
	}//end for

	std::cout << "Total get " << poseCount << " frames." << std::endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

#ifdef _WIN32

float Application::TimeCost(SYSTEMTIME &sys_prev, SYSTEMTIME &sys_later)
{
	float cost1 = (sys_later.wDay - sys_prev.wDay) * 24 * 60 * 60 * 1000;
	float cost2 = (sys_later.wHour - sys_prev.wHour) * 60 * 60 * 1000;
	float cost3 = (sys_later.wMinute - sys_prev.wMinute) * 60 * 1000;
	float cost4 = (sys_later.wSecond - sys_prev.wSecond) * 1000;
	float cost5 = (sys_later.wMilliseconds - sys_prev.wMilliseconds);
	float totalCost = cost1 + cost2 + cost3 + cost4 + cost5;

	return totalCost;
}

#endif

///////////////////////////////////////////////////////////////////////////////////////////////////

void Application::FromQuanternion2Matrix(TUMPose &tum_pose, cv::Mat &camera_pose)
{
	if (!tum_pose.mTS)
	{
		std::cerr << "Error: no input tum pose." << std::endl;
		return;
	}//end if

	camera_pose.create(4, 4, CV_32FC1);
	camera_pose.setTo(0);

	float xx = tum_pose.mQX * tum_pose.mQX;
	float yy = tum_pose.mQY * tum_pose.mQY;
	float zz = tum_pose.mQZ * tum_pose.mQZ;
	float ww = tum_pose.mQW * tum_pose.mQW;

	if ( (xx + yy + zz + ww)!=1.0 )
	{
		float base = 1 / std::sqrt( tum_pose.mQX * tum_pose.mQX + tum_pose.mQY * tum_pose.mQY + tum_pose.mQZ * tum_pose.mQZ + tum_pose.mQW * tum_pose.mQW );
		tum_pose.mQX = tum_pose.mQX * base;
		tum_pose.mQY = tum_pose.mQY * base;
		tum_pose.mQZ = tum_pose.mQZ * base;
		tum_pose.mQW = tum_pose.mQW * base;

		xx = tum_pose.mQX * tum_pose.mQX;
		yy = tum_pose.mQY * tum_pose.mQY;
		zz = tum_pose.mQZ * tum_pose.mQZ;
		ww = tum_pose.mQW * tum_pose.mQW;
	}//end if

	float xw = tum_pose.mQX * tum_pose.mQW;
	float xy = tum_pose.mQX * tum_pose.mQY;
	float xz = tum_pose.mQX * tum_pose.mQZ;
	float yz = tum_pose.mQY * tum_pose.mQZ;
	float yw = tum_pose.mQY * tum_pose.mQW;
	float zw = tum_pose.mQZ * tum_pose.mQW;

	camera_pose.at<float>(0, 0) = 1 - 2 * (yy + zz);
	camera_pose.at<float>(0, 1) = 2 * (xy - zw);
	camera_pose.at<float>(0, 2) = 2 * (xz + yw);
	camera_pose.at<float>(1, 0) = 2 * (xy + zw);
	camera_pose.at<float>(1, 1) = 1 - 2 * (xx + zz);
	camera_pose.at<float>(1, 2) = 2 * (yz - xw);
	camera_pose.at<float>(2, 0) = 2 * (xz - yw);
	camera_pose.at<float>(2, 1) = 2 * (yz + xw);
	camera_pose.at<float>(2, 2) = 1 - 2 * (xx + yy);

	camera_pose.at<float>(0, 3) = tum_pose.mTX;
	camera_pose.at<float>(1, 3) = tum_pose.mTY;
	camera_pose.at<float>(2, 3) = tum_pose.mTZ;

	camera_pose.at<float>(3, 0) = 0;
	camera_pose.at<float>(3, 1) = 0;
	camera_pose.at<float>(3, 2) = 0;
	camera_pose.at<float>(3, 3) = 1;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Application::InitDefaultSetting()
{
	std::cout<<"Getting default setting......"<<std::endl;
	GetDefaultSetting();
	std::cout<<"Get default setting, done."<<std::endl;

	std::cout<<std::endl<<"Initialing......"<<std::endl;

	if (mVoxelType.compare("geometry") == 0)
	{
		TSDFGeometry *tsdfGeo = new TSDFGeometry();
		tsdfGeo->SetTruncateDist( mTruncateDist );
		tsdfGeo->SetSaveThresh( mSaveThresh );
		mTsdfPtr.reset( tsdfGeo );
		//mTsdfPtr.reset(new TSDFGeometry);
		mTsdfPtr->SetResolution(mResX, mResY, mResZ);
	}//end if
	else if (mVoxelType.compare("volumetric") == 0)
	{
		TSDFVolumetricFusion *tsdfVFusion = new TSDFVolumetricFusion();
		tsdfVFusion->SetTruncateDist( mTruncateDist );
		tsdfVFusion->SetSaveThresh( mSaveThresh );
		mTsdfPtr.reset( tsdfVFusion );
		mTsdfPtr->SetResolution(mResX, mResY, mResZ);
	}//end else if
	else
	{
		TSDFTexture *tsdfTex = new TSDFTexture();
		tsdfTex->SetTheta( mTheta );
		mTsdfPtr.reset( tsdfTex );
		//mTsdfPtr.reset(new TSDFTexture);
		mTsdfPtr->SetResolution(mResX*2, mResY*2, mResZ*2);
	}//end else

	//mTsdfPtr->SetResolution(mResX, mResY, mResZ);
	mTsdfPtr->SetSize(mSizeX, mSizeY, mSizeZ);
	mTsdfPtr->SetCamera(mFX, mFY, mCX, mCY, mFactor);
	mTsdfPtr->SetSensorRange(mRangeMin, mRangeMax);
	mTsdfPtr->InitVolume();

	std::cout<<"Initialize done."<<std::endl<<std::endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Application::GetDefaultSetting()
{
	std::string defaultFile = "data/defaultParameters.xml";

	cv::FileStorage fs;
	fs.open(defaultFile, cv::FileStorage::READ);
	if ( !fs.isOpened() )
	{
		std::cerr<<"No such file. We will create the default parameters files."<<std::endl;
		fs.open(defaultFile, cv::FileStorage::WRITE);

		fs<<"fx"<<525.0;
		fs<<"fy"<<525.0;
		fs<<"cx"<<319.5;
		fs<<"cy"<<239.5;
		fs<<"factor"<<5000.0;
		fs<<"truncateDist"<<0.08;
		fs<<"theta"<<0.025;
		fs<<"resX"<<200;
		fs<<"resY"<<200;
		fs<<"resZ"<<200;
		fs<<"sizeX"<<2.0;
		fs<<"sizeY"<<2.0;
		fs<<"sizeZ"<<2.0;
		fs<<"frameNum"<<10;
		fs<<"rangeMin"<<0.4;
		fs<<"rangeMax"<<3.0;
		fs<<"maxWeight"<<50;
		fs<<"saveThresh"<<0.5;
		fs<<"voxelType"<<"geometry";
		fs<<"saveTsdf"<<"saveTsdf";
		fs<<"toMesh"<<"m";
	}//end if

	fs.open(defaultFile, cv::FileStorage::READ);
	if ( !fs.isOpened() )
	{
		std::cerr<<"Error: cannot open the file."<<std::endl;

		mFX = 525.0;
		mFY = 525.0;
		mCX = 319.5;
		mCY = 239.5;
		mFactor = 5000.;
		mTruncateDist = 0.08;
		mTheta = 0.025;
		mResX = 200;
		mResY = 200;
		mResZ = 200;
		mSizeX = 2.0;
		mSizeY = 2.0;
		mSizeZ = 2.0;
		mFrameNum = 10;
		mRangeMin = 0.4;
		mRangeMax = 3.0;
		mMaxWeight = 50.0;
		mSaveThresh = 0.5;
		mVoxelType = "geometry";
		mSaveTsdf = "savaTsdf";
		mToMesh = "m";

		return;
	}//end if

	mFX = (float)fs["fx"];
	mFY = (float)fs["fy"];
	mCX = (float)fs["cx"];
	mCY = (float)fs["cy"];
	mFactor = (float)fs["factor"];
	mTruncateDist = (float)fs["truncateDist"];
	mTheta = (float)fs["theta"];
	mResX = (int)fs["resX"];
	mResY = (int)fs["resY"];
	mResZ = (int)fs["resZ"];
	mSizeX = (float)fs["sizeX"];
	mSizeY = (float)fs["sizeY"];
	mSizeZ = (float)fs["sizeZ"];
	mFrameNum= (int)fs["frameNum"];
	mRangeMin = (float)fs["rangeMin"];
	mRangeMax = (float)fs["rangeMax"];
	mMaxWeight = (float)fs["maxWeight"];
	mSaveThresh = (float)fs["saveThresh"];
	mVoxelType = (std::string)fs["voxelType"];
	mSaveTsdf = (std::string)fs["saveTsdf"];
	mToMesh = (std::string)fs["toMesh"];

	fs.release();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Application::SetParameter(std::shared_ptr<TSDFBase> tsdf)
{
	
}

///////////////////////////////////////////////////////////////////////////////////////////////////

std::string Application::GetDataByName(std::string node_name)
{
	cv::FileStorage fs;
	fs.open( "data/defaultParameters.xml", cv::FileStorage::READ );
	if ( !fs.isOpened() )
	{
		std::cerr<<"ERROR - 3 Can not open the file."<<std::endl;
		return std::string("ERROR");
	}// end if

	//std::string str = (std::string)fs[node_name];
	auto ret = fs[node_name];
	std::stringstream ss;
	int iRet;
	double dRet;
	std::string sRet;
	if ( ret.isInt() )
	{
		iRet = (int)ret;
		ss<<iRet;
		ss>>sRet;
	}// end if
	else if ( ret.isString() )
	{
		sRet = (std::string)ret;
	}// end else if
	else if ( ret.isReal() )
	{
		dRet = (double)ret;
		ss<<dRet;
		ss>>sRet;
	}// end else if
	else
	{
#ifdef _DEBUG
		std::cout<<"ERROR - Wrong type."<<std::endl;
#endif
		sRet = "ERROR";
	}// end else

	return sRet;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Application::MatrixXMatrix(cv::Mat left, cv::Mat right, cv::Mat &output)
{
	output.at<float>(0, 0) = left.at<float>(0, 0) * right.at<float>(0, 0) + left.at<float>(0, 1) * right.at<float>(1, 0) + left.at<float>(0, 2) * right.at<float>(2, 0);
	output.at<float>(1, 0) = left.at<float>(1, 0) * right.at<float>(0, 0) + left.at<float>(1, 1) * right.at<float>(1, 0) + left.at<float>(1, 2) * right.at<float>(2, 0);
	output.at<float>(2, 0) = left.at<float>(2, 0) * right.at<float>(0, 0) + left.at<float>(2, 1) * right.at<float>(1, 0) + left.at<float>(2, 2) * right.at<float>(2, 0);

	output.at<float>(0, 1) = left.at<float>(0, 0) * right.at<float>(0, 1) + left.at<float>(0, 1) * right.at<float>(1, 1) + left.at<float>(0, 2) * right.at<float>(2, 1);
	output.at<float>(1, 1) = left.at<float>(1, 0) * right.at<float>(0, 1) + left.at<float>(1, 1) * right.at<float>(1, 1) + left.at<float>(1, 2) * right.at<float>(2, 1);
	output.at<float>(2, 1) = left.at<float>(2, 0) * right.at<float>(0, 1) + left.at<float>(2, 1) * right.at<float>(1, 1) + left.at<float>(2, 2) * right.at<float>(2, 1);

	output.at<float>(0, 2) = left.at<float>(0, 0) * right.at<float>(0, 2) + left.at<float>(0, 1) * right.at<float>(1, 2) + left.at<float>(0, 2) * right.at<float>(2, 2);
	output.at<float>(1, 2) = left.at<float>(1, 0) * right.at<float>(0, 2) + left.at<float>(1, 1) * right.at<float>(1, 2) + left.at<float>(1, 2) * right.at<float>(2, 2);
	output.at<float>(2, 2) = left.at<float>(2, 0) * right.at<float>(0, 2) + left.at<float>(2, 1) * right.at<float>(1, 2) + left.at<float>(2, 2) * right.at<float>(2, 2);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Application::TestTexture()
{
	/// 0. First load ply file.
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr meshCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if ( pcl::io::loadPLYFile("my_mesh_for_texture.ply", *meshCloud)<0 )
	{
		std::cerr<<"Error: cannot read the file <myMesh_better_more.ply>."<<std::endl;
		return;
	}//end if
	std::cout<<"mesh cloud size: "<<meshCloud->points.size()<<std::endl;

	pcl::visualization::CloudViewer viewer("debug");
	viewer.showCloud( meshCloud );
	while ( !viewer.wasStopped() )
	{
	}//end while.

	/// 1. Second calculate the color volume.
	std::cout << std::endl << "Using default setting." << std::endl << std::endl;

	// 1.0. Init default setting.
	InitDefaultSetting();

	// 1.1. Get default data.
	std::cout << "Getting data......" << std::endl;
	std::vector<cv::Mat> depthImgs, rgbImgs;
	std::vector<TUMPose> tumPoses;
	GetDefaultData(depthImgs, rgbImgs, tumPoses);
	std::cout << "Get data, done." << std::endl;

	// Check data.
	if (depthImgs.empty() || depthImgs.size() != rgbImgs.size() || rgbImgs.size() != tumPoses.size())
	{
		std::cerr << "Error: no data or invalid data." << std::endl;
		return;
	}//end if

	// 1.2. For each frame, do volumetric fusion.
#ifdef _WIN32
	SYSTEMTIME sysPrev, sysLater;
#else
	struct timeval prevTime, laterTime;
#endif
	for (size_t i = 0; i < depthImgs.size(); ++i)
	{
		std::cout << std::endl << "Processing frame: " << i + 1 << std::endl;
#ifdef _WIN32
		GetLocalTime(&sysPrev);
#else
		gettimeofday(&prevTime, NULL);
#endif
		// 1.3. Calculate camera pose.
		cv::Mat camPose;
		FromQuanternion2Matrix(tumPoses[i], camPose);

		// Check data.
		if (camPose.empty())
		{
			std::cout << "Warning: pose is empty of frame " << i + 1 << std::endl;
			continue;
		}//end if

		cv::Mat invCamPose = camPose.inv();

		if ( invCamPose.empty() )
		{
			std::cout<< "Warning: inv pose is empty in the frame "<< i+ 1 <<std::endl;
			continue;
		}//end if

		// 1.4. Do volumetric fusion.
		mTsdfPtr->UpdateVolume(depthImgs[i], rgbImgs[i], invCamPose);
			
#ifdef _WIN32
		GetLocalTime(&sysLater);
#else
		gettimeofday(&laterTime, NULL);
#endif
		std::cout << "Process frame " << i + 1 << " done." << std::endl;
#ifdef _WIN32
		std::cout << "Time cost: " << TimeCost(sysPrev, sysLater) << " ms." << std::endl;
#else
		double timeCost = (laterTime.tv_sec - prevTime.tv_sec) * 1000 + (laterTime.tv_usec - prevTime.tv_usec) / 1000;
		std::cout << "Time cost: " << timeCost << " ms." << std::endl;
#endif
	}//end for

	TestSaveColorVolumeResult( meshCloud );

	/// 2. Then, generate the textures.
	TestGenerateTexture(meshCloud, mTsdfPtr);

	/// 3. And save the result.
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Application::TestGenerateTexture(pcl::PointCloud<pcl::PointXYZRGB>::Ptr mesh_cloud, std::shared_ptr<TSDFBase> tsdf_ptr)
{
	std::cout<<"Generating the texture......"<<std::endl;
	/// For each triangle in the mesh, calculate the texture from color volume.
	int resX = mResX * 2;
	int resY = mResY * 2;
	int resZ = mResZ * 2;
	float pixelSize = mSizeX / resX;
	cv::Mat texture(480, 600, CV_8UC3, cv::Scalar::all( 255 ));
	int currRow = 0;
	int currCol = 0;
	int maxCol = 0;

	std::vector<cv::Mat> textures;
	std::vector<int    > pointIdx;
	std::vector<cv::Point2i> coordInMat;
	std::vector<cv::Point2f> textureCoord;

	float stepX = mSizeX / resX;
	float stepY = mSizeY / resY;
	float stepZ = mSizeZ / resZ;

	for (size_t i = 0; i<mesh_cloud->points.size(); i+=3)
	{
		pcl::PointXYZRGB pt1, pt2, pt3;
		pt1 = mesh_cloud->points[i+0];
		pt2 = mesh_cloud->points[i+1];
		pt3 = mesh_cloud->points[i+2];

		float dist12 = std::sqrt( (pt1.x - pt2.x) * (pt1.x - pt2.x) + (pt1.y - pt2.y) * (pt1.y - pt2.y) + (pt1.z - pt2.z) * (pt1.z - pt2.z) );
		float dist13 = std::sqrt( (pt1.x - pt3.x) * (pt1.x - pt3.x) + (pt1.y - pt3.y) * (pt1.y - pt3.y) + (pt1.z - pt3.z) * (pt1.z - pt3.z) );

		int w = dist12 / pixelSize + 0.50;
		int h = dist13 / pixelSize + 0.50;

		if ( w<=1 )
			w = 2;
		if ( h<=1 )
			h = 2;

		if ( (currRow + h)>480 )
		{
			currCol += maxCol;
			currRow = 0;
			maxCol = 0;
		}//end if
		if ( (currCol + w)>640 )
		{
			textures.push_back( texture );
			pointIdx.push_back( i );
			texture.setTo( 255 );
			currCol = 0;
			currRow = 0;
			maxCol = 0;
		}//end if
		if ( w>maxCol )
			maxCol = w;

		cv::Point3f unitVec12, unitVec13;
		unitVec12.x = (pt2.x - pt1.x) / (w - 1);
		unitVec12.y = (pt2.y - pt1.y) / (w - 1);
		unitVec12.z = (pt2.z - pt1.z) / (w - 1);
		unitVec13.x = (pt3.x - pt1.x) / (h - 1);
		unitVec13.y = (pt3.y - pt1.y) / (h - 1);
		unitVec13.z = (pt3.z - pt1.z) / (h - 1);

		for (int r = 0; r < h; ++r)
		{
			for (int c = 0; c < w; ++c)
			{
				cv::Point3f pos;
				pos.x = pt1.x + r * unitVec13.x + c * unitVec12.x;
				pos.y = pt1.y + r * unitVec13.y + c * unitVec12.y;
				pos.z = pt1.z + r * unitVec13.z + c * unitVec12.z;

				int idxX = - pos.x / stepX;
				int idxY =   pos.y / stepY;
				int idxZ =   pos.z / stepZ;

				int index = idxX + idxY * resX + idxZ * resX * resY;
				uchar red, green, blue;
				tsdf_ptr->GetRGB(index, red, green, blue);

				texture.at<cv::Vec3b>(currRow, currCol)[0] = blue;
				texture.at<cv::Vec3b>(currRow, currCol)[1] = green;
				texture.at<cv::Vec3b>(currRow, currCol)[2] = red;

				currCol++;
			}//end for

			currCol -= w;
			currRow++;
		}//end for

		cv::Point2i pt1rc, pt2rc, pt3rc;
		pt1rc.x = currCol;
		pt1rc.y = currRow - h;
		pt2rc.x = currCol + w - 1;
		pt2rc.y = currRow - h;
		pt3rc.x = currCol;
		pt3rc.y = currRow - 1;
		coordInMat.push_back( pt1rc );
		coordInMat.push_back( pt2rc );
		coordInMat.push_back( pt3rc );
	}//end for

	textures.push_back( texture );
	pointIdx.push_back( mesh_cloud->points.size()-1 );

	if ( textures.size()<1 )
	{
		std::cerr<<"Error: not texture generated."<<std::endl;
		return;
	}//end if

	if ( textures.size()>1 )
	{
		cv::Mat finalTexture;

		TestStitchAndUpdate(finalTexture, coordInMat, textures, currCol, pointIdx);

		if ( finalTexture.empty() )
		{
			std::cerr<<"Error: stitching texture error, no texture data."<<std::endl;
			return;
		}//end if

		unsigned int textureRow = finalTexture.rows;
		unsigned int textureCol = finalTexture.cols;

		for (size_t i = 0; i < coordInMat.size(); ++i)
		{
			int x = coordInMat[i].x;
			int y = textureRow - coordInMat[i].y;

			cv::Point2f texPt;
			texPt.x = (float)(x) / (float)(textureCol);
			texPt.y = (float)(y) / (float)(textureRow);

			textureCoord.push_back( texPt );
		}//end for

		cv::imwrite("finalTexture.png", finalTexture);

		for (int i=0; i<textures.size(); ++i)
		{
			std::ostringstream ostr;
			ostr << i;
			std::string texName = "textures" + ostr.str() + ".png";
			cv::imwrite(texName, textures[i]);
		}//end for
	}//end if
	else if ( textures.size()==1 )
	{
		int textureRow = textures[0].rows;
		int textureCol = textures[0].cols;

		std::cout<<"coordInMat size: "<<coordInMat.size()<<std::endl;

		for (size_t i = 0; i < coordInMat.size(); ++i)
		{
			int x = coordInMat[i].x;
			int y = textureRow - coordInMat[i].y;

			cv::Point2f texPt;
			if ( i%3==0 )
			{
				texPt.x = (float)(x+0.5) / (float)(textureCol);
				texPt.y = (float)(y-0.5) / (float)(textureRow);
			}//end if
			else if ( i%3==1 )
			{
				texPt.x = (float)(x-0.5) / (float)(textureCol);
				texPt.y = (float)(y-0.5) / (float)(textureRow);
			}//end else if
			else if ( i%3==2 )
			{
				texPt.x = (float)(x+0.5) / (float)(textureCol);
				texPt.y = (float)(y+0.5) / (float)(textureRow);
			}//end else if
			else
			{
				texPt.x = (float)(x) / (float)(textureCol);
				texPt.y = (float)(y) / (float)(textureRow);
			}//end else

			textureCoord.push_back( texPt );
		}//end for

		cv::imwrite("finalTexture.png", textures[0]);
	}//end else if
	std::cout<<"Generate texture, done."<<std::endl;

	TestSaveResult(mesh_cloud, textureCoord);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Application::TestStitchAndUpdate(cv::Mat &texture, std::vector<cv::Point2i> &coord, std::vector<cv::Mat> &textures, int curr_col, std::vector<int> &point_idx)
{
	int textureRow = textures[0].rows;
	int textureCol = (textures.size() -1) * textures[0].cols + curr_col;

	texture.create(textureRow, textureCol, CV_8UC3);
	texture.setTo( 0 );

	int originCol = textures[0].cols;
	int originRow = textures[0].rows;

	for (int i = 0; i < textures.size() - 1; ++i)
	{
		int beginCol = i * originCol;
		cv::Mat roi(texture, cv::Rect(beginCol, 0, originCol, originRow));
		textures[i].copyTo( roi );
	}//end for

	int index = textures.size() - 1;
	cv::Mat roi(texture, cv::Rect((index * originCol), 0, curr_col, originRow));
	textures[index](cv::Range::all(), cv::Range(0, curr_col)).copyTo( roi );

	for (int i=0; i<point_idx.size()-1; ++i)
	{
		for (int n = point_idx[i]; n < point_idx[i+1]; ++n)
		{
			coord[n].x += ((i+1)*originCol);
		}//end for
	}//end for
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Application::TestSaveResult(pcl::PointCloud<pcl::PointXYZRGB>::Ptr mesh_cloud, std::vector<cv::Point2f> &tex_coord)
{
	std::cout<<"Saving the mesh with texture......"<<std::endl;
	std::ofstream outfile;
	outfile.open("my_mesh_with_texture_result.ply");
	if ( !outfile.is_open() )
	{
		std::cerr<<"Cannot open the file: my_mesh_with_texture_result.ply"<<std::endl;
		return;
	}//end if

	outfile<<"ply"<<std::endl;
	outfile<<"format ascii 1.0"<<std::endl;
	outfile<<"comment PCL generated"<<std::endl;
	outfile<<"comment TextureFile my_texture.png"<<std::endl;
	outfile<<"element vertex "<<mesh_cloud->points.size()<<std::endl;
	outfile<<"property float x"<<std::endl;
	outfile<<"property float y"<<std::endl;
	outfile<<"property float z"<<std::endl;
	outfile<<"element face "<<mesh_cloud->points.size()/3<<std::endl;
	outfile<<"property list uchar int vertex_index"<<std::endl;
	outfile<<"property list uchar float texcoord"<<std::endl;
	outfile<<"end_header"<<std::endl;

	for (int i=0; i<mesh_cloud->points.size(); ++i)
	{
		outfile<<mesh_cloud->points[i].x<<" "<<mesh_cloud->points[i].y<<" "<<mesh_cloud->points[i].z<<std::endl;
	}//end for

	for (int i=0; i<tex_coord.size(); i+=3)
	{
		outfile<<3<<" "<<(i+2)<<" "<<(i+1)<<" "<<i<<" "<<6<<" "<<tex_coord[i+2].x<<" "<<tex_coord[i+2].y<<" ";
		outfile<<std::setprecision(5)<<tex_coord[i+1].x<<" "<<tex_coord[i+1].y<<" "<<tex_coord[i].x<<" "<<tex_coord[i].y<<std::endl;
	}//end for

	outfile.close();

	std::cout<<"Save the mesh with texture done."<<std::endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Application::TestSaveColorVolumeResult(pcl::PointCloud<pcl::PointXYZRGB>::Ptr mesh_cloud)
{
	Eigen::Vector4f minPt, maxPt;
	pcl::getMinMax3D(*mesh_cloud, minPt, maxPt);

	int resX = mResX * 2;
	int resY = mResY * 2;
	int resZ = mResZ * 2;
	
	float stepX = mSizeX / (mResX * 2);
	float stepY = mSizeY / (mResY * 2);
	float stepZ = mSizeZ / (mResZ * 2);

	int minIdxX = -maxPt.x() / stepX;
	int minIdxY =  minPt.y() / stepY;
	int minIdxZ =  minPt.z() / stepZ;
	int maxIdxX = -minPt.x() / stepX;
	int maxIdxY =  maxPt.y() / stepY;
	int maxIdxZ =  maxPt.z() / stepZ;

	std::cout<<"minx: "<<minIdxX<<" , miny: "<<minIdxY<<" , minz: "<<minIdxZ<<std::endl;
	std::cout<<"maxx: "<<maxIdxX<<" , maxy: "<<maxIdxY<<" , maxz: "<<maxIdxZ<<std::endl;
	std::cout<<"stepx:"<<stepX  <<" , stepy:"<<stepY  <<" , stepz:"<<stepZ  <<std::endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud( mesh_cloud );

	for (int x = minIdxX; x < maxIdxX; ++x)
	{
		for (int y = minIdxY; y < maxIdxY; ++y)
		{
			for (int z = minIdxZ; z < maxIdxZ; ++z)
			{
				pcl::PointXYZRGB pt;
				pt.x = -(x + 0.5) * stepX;
				pt.y =  (y + 0.5) * stepY;
				pt.z =  (z + 0.5) * stepZ;

				int index = x + y * resX + z * resX * resY;
				uchar red, green, blue;
				mTsdfPtr->GetRGB(index, red, green, blue);

				pt.r = (int)red;
				pt.g = (int)green;
				pt.b = (int)blue;

				std::vector<int> nn_indices;
				std::vector<float> nn_dists;

				tree->nearestKSearchT(pt, 1, nn_indices, nn_dists);

				//std::cout<<"pt.x: "<<pt.x<<", pt.y: "<<pt.y<<", pt.z: "<<pt.z<<std::endl;
				//std::cout<<"mesh_cloud.x: "<<mesh_cloud->points[nn_indices[0]].x<<", mesh_cloud.y: "<<mesh_cloud->points[nn_indices[0]].y<<", mesh_cloud.z: "<<mesh_cloud->points[nn_indices[0]].z<<std::endl;

				if ( nn_dists[0]>0.002 )
					continue;

				colorCloud->points.push_back( pt );

				/*std::cout<<"pt.x: "<<pt.x<<", pt.y: "<<pt.y<<", pt.z: "<<pt.z<<std::endl;
				std::cout<<"mesh_cloud.x: "<<mesh_cloud->points[nn_indices[0]].x<<", mesh_cloud.y: "<<mesh_cloud->points[nn_indices[0]].y<<", mesh_cloud.z: "<<mesh_cloud->points[nn_indices[0]].z<<std::endl;
				std::cout<<"r: "<<int( pt.r )<<", g: "<<int( pt.g )<<", b: "<<int( pt.b )<<std::endl;
				std::cout<<"w: "<<mTsdfPtr->GetWeight(index)<<std::endl;*/
			}//end for
		}//end for
	}//end for

	colorCloud->width = colorCloud->points.size();
	colorCloud->height = 1;
	colorCloud->is_dense = false;
	colorCloud->points.resize( colorCloud->width * colorCloud->height );

	std::cout<<"point cloud size: "<<colorCloud->points.size()<<std::endl;

	pcl::visualization::CloudViewer viewer("debug");
	viewer.showCloud( colorCloud );
	while ( !viewer.wasStopped() )
	{
	}//end while.

	pcl::io::savePCDFileASCII("colorCloud.pcd", *colorCloud);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void Application::Texture()
{
	TextureGenerator tg;
	tg.SetColorResolution(400, 400, 400);
	tg.SetColorSize(2.0, 2.0, 2.0);
	tg.SetGeometryResolution(100, 100, 100);
	tg.SetGeometrySize(2.0, 2.0, 2.0);
	tg.Generate();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

