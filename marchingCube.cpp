#include "marchingCube.h"

///////////////////////////////////////////////////////////////////////////////////////////////////

MarchingCube::MarchingCube()
	: pcl::MarchingCubes<pcl::PointXYZRGB>()
	, mMinWeight( 2.0 )
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

MarchingCube::~MarchingCube()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void MarchingCube::voxelizeData()
{
	float sizeX = 0, sizeY = 0, sizeZ = 0;
	int resX = 0, resY = 0, resZ = 0;
	mTsdfPtr->GetSize(sizeX, sizeY, sizeZ);
	mTsdfPtr->GetResolution(resX, resY, resZ);

	float stepX = sizeX / resX;
	float stepY = sizeY / resY;
	float stepZ = sizeZ / resZ;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

	for (int x = 0; x <= res_x_; ++x)
	{
		for (int y = 0; y <= res_y_; ++y)
		{
			for (int z = 0; z <= res_z_; ++z)
			{
				std::vector<int> nn_indices;
				std::vector<float> nn_sqrt_dists;

				Eigen::Vector3f point;
				point[0] = min_p_[0] + (max_p_[0] - min_p_[0]) * float( x ) / float( res_x_ );
				point[1] = min_p_[1] + (max_p_[1] - min_p_[1]) * float( y ) / float( res_y_ );
				point[2] = min_p_[2] + (max_p_[2] - min_p_[2]) * float( z ) / float( res_z_ );

				pcl::PointXYZRGB p;
				//p.getVector3fMap() = point;
				p.x = point[0];
				p.y = point[1];
				p.z = point[2];

				tree_->nearestKSearch(p, 1, nn_indices, nn_sqrt_dists);

				Eigen::Vector3f nnPt = input_->points[ nn_indices[0] ].getVector3fMap();
				int idxX = -nnPt.x() / stepX ;
				int idxY =  nnPt.y() / stepY ;
				int idxZ =  nnPt.z() / stepZ ;

				/*std::cout<<"idxX(float): "<<-nnPt.x() / stepX - 0.5<<", idxY(float): "<<nnPt.y() / stepY - 0.5<<", idxZ(float): "<<nnPt.z() / stepZ - 0.5<<std::endl;
				std::cout<<"idxX(int): "<<idxX<<", idxY(int): "<<idxY<<", idxZ(int): "<<idxZ<<std::endl;*/

				int index = idxX + idxY * resX + idxZ * resX * resY;

				int gridIdx = x * res_y_ * res_z_ + y * res_z_ + z;

				//if ( mTsdfPtr->GetWeight(index)<mMinWeight )
				//if ( nn_sqrt_dists[0]>0.000001 )
				if ( std::abs(p.x - nnPt[0])>=0.001 || std::abs(p.y - nnPt[1])>=0.001 || std::abs(p.z - nnPt[2])>=0.001 )
				{
					grid_[ gridIdx ] = 0.0;
					continue;
				}//end if

				/*std::cout<<"p.x: "<<p.x<<", p.y: "<<p.y<<" , p.z: "<<p.z<<std::endl;
				std::cout<<"nnPt[0]: "<<nnPt[0]<<" , nnPt[1]: "<<nnPt[1]<<" , nnPt[2]: "<<nnPt[2]<<std::endl;*/
				/*std::cout<<"resolution: "<<res_x_<<" "<<res_y_<<" "<<res_z_<<std::endl;
				std::cout<<"voxel point: "<<point[0]<<" "<<point[1]<<" "<<point[2]<<std::endl;
				std::cout<<"nearest point: "<<nnPt.x()<<" "<<nnPt.y()<<" "<<nnPt.z()<<std::endl;
				std::cout<<"min_pt: "<<min_p_[0]<<" "<<min_p_[1]<<" "<<min_p_[2]<<", max_p: "<<max_p_[0]<<" "<<max_p_[1]<<" "<<max_p_[2]<<std::endl;
				std::cout<<std::endl;*/

				uchar r, g, b;
				mTsdfPtr->GetRGB(index, r, g, b);

				p.r = b;
				p.g = g;
				p.b = r;

				cloud->points.push_back( p );

				grid_[ gridIdx ] = mTsdfPtr->GetTsdfValue(index) + 0.500;
			}//end inner for
		}//end for
	}//end for

	cloud->width = cloud->points.size();
	cloud->height = 1;
	cloud->is_dense = false;
	cloud->resize( cloud->width * cloud->height );

	std::cout<<"input cloud size: "<<input_->points.size()<<std::endl;
	std::cout<<"Point number: "<<cloud->points.size()<<std::endl;
	pcl::visualization::CloudViewer viewer("Debug Viewer");
	viewer.showCloud( cloud );
	while ( !viewer.wasStopped() )
	{
	}//end while
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void MarchingCube::interpolateEdge(Eigen::Vector3f &p1, Eigen::Vector3f &p2, float val_p1, float val_p2, Eigen::Vector3f &out_pt, Eigen::Vector3i &out_color)
{
	float mu = (iso_level_ - val_p1)/(val_p2 - val_p1);
	out_pt = p1 + mu * (p2 - p1);

	// Interpolate color.
	pcl::PointXYZRGB point1, point2;

	point1.x = p1[0];
	point1.y = p1[1];
	point1.z = p1[2];
	point2.x = p2[0];
	point2.y = p2[1];
	point2.z = p2[2];

	std::vector<int> nn_indices1, nn_indices2;
	std::vector<float> nn_sqr_dists1, nn_sqr_dists2;

	tree_->nearestKSearch (point1, 1, nn_indices1, nn_sqr_dists1);
	tree_->nearestKSearch (point2, 1, nn_indices2, nn_sqr_dists2);

	Eigen::Vector3i color1, color2;

	color1[0] = input_->points[ nn_indices1[0] ].r;
	color1[1] = input_->points[ nn_indices1[0] ].g;
	color1[2] = input_->points[ nn_indices1[0] ].b;
	color2[0] = input_->points[ nn_indices2[0] ].r;
	color2[1] = input_->points[ nn_indices2[0] ].g;
	color2[2] = input_->points[ nn_indices2[0] ].b;

	out_color = color1 + mu * (color2 - color1);
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void MarchingCube::createSurface(std::vector<float> &leaf_node, Eigen::Vector3i &index_3d, pcl::PointCloud<pcl::PointXYZRGB> &cloud)
{
	int cubeindex = 0;
	Eigen::Vector3f vertex_list[12];
	Eigen::Vector3i color_list[12];
	if (leaf_node[0] < iso_level_) cubeindex |= 1;
	if (leaf_node[1] < iso_level_) cubeindex |= 2;
	if (leaf_node[2] < iso_level_) cubeindex |= 4;
	if (leaf_node[3] < iso_level_) cubeindex |= 8;
	if (leaf_node[4] < iso_level_) cubeindex |= 16;
	if (leaf_node[5] < iso_level_) cubeindex |= 32;
	if (leaf_node[6] < iso_level_) cubeindex |= 64;
	if (leaf_node[7] < iso_level_) cubeindex |= 128;

	// Cube is entirely in/out of the surface
	if (pcl::edgeTable[cubeindex] == 0)
		return;

	//Eigen::Vector4f index_3df (index_3d[0], index_3d[1], index_3d[2], 0.0f);
	Eigen::Vector3f center;// TODO coeff wise product = min_p_ + Eigen::Vector4f (1.0f/res_x_, 1.0f/res_y_, 1.0f/res_z_) * index_3df * (max_p_ - min_p_);
	center[0] = min_p_[0] + (max_p_[0] - min_p_[0]) * float (index_3d[0]) / float (res_x_);
	center[1] = min_p_[1] + (max_p_[1] - min_p_[1]) * float (index_3d[1]) / float (res_y_);
	center[2] = min_p_[2] + (max_p_[2] - min_p_[2]) * float (index_3d[2]) / float (res_z_);

	std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f> > p;
	p.resize (8);
	for (int i = 0; i < 8; ++i)
	{
		Eigen::Vector3f point = center;
		if(i & 0x4)
			point[1] = static_cast<float> (center[1] + (max_p_[1] - min_p_[1]) / float (res_y_));

		if(i & 0x2)
			point[2] = static_cast<float> (center[2] + (max_p_[2] - min_p_[2]) / float (res_z_));

		if((i & 0x1) ^ ((i >> 1) & 0x1))
			point[0] = static_cast<float> (center[0] + (max_p_[0] - min_p_[0]) / float (res_x_));

		p[i] = point;
	}


	// Find the vertices where the surface intersects the cube
	if (pcl::edgeTable[cubeindex] & 1)
		interpolateEdge (p[0], p[1], leaf_node[0], leaf_node[1], vertex_list[0], color_list[0]);
	if (pcl::edgeTable[cubeindex] & 2)
		interpolateEdge (p[1], p[2], leaf_node[1], leaf_node[2], vertex_list[1], color_list[1]);
	if (pcl::edgeTable[cubeindex] & 4)
		interpolateEdge (p[2], p[3], leaf_node[2], leaf_node[3], vertex_list[2], color_list[2]);
	if (pcl::edgeTable[cubeindex] & 8)
		interpolateEdge (p[3], p[0], leaf_node[3], leaf_node[0], vertex_list[3], color_list[3]);
	if (pcl::edgeTable[cubeindex] & 16)
		interpolateEdge (p[4], p[5], leaf_node[4], leaf_node[5], vertex_list[4], color_list[4]);
	if (pcl::edgeTable[cubeindex] & 32)
		interpolateEdge (p[5], p[6], leaf_node[5], leaf_node[6], vertex_list[5], color_list[5]);
	if (pcl::edgeTable[cubeindex] & 64)
		interpolateEdge (p[6], p[7], leaf_node[6], leaf_node[7], vertex_list[6], color_list[6]);
	if (pcl::edgeTable[cubeindex] & 128)
		interpolateEdge (p[7], p[4], leaf_node[7], leaf_node[4], vertex_list[7], color_list[7]);
	if (pcl::edgeTable[cubeindex] & 256)
		interpolateEdge (p[0], p[4], leaf_node[0], leaf_node[4], vertex_list[8], color_list[8]);
	if (pcl::edgeTable[cubeindex] & 512)
		interpolateEdge (p[1], p[5], leaf_node[1], leaf_node[5], vertex_list[9], color_list[9]);
	if (pcl::edgeTable[cubeindex] & 1024)
		interpolateEdge (p[2], p[6], leaf_node[2], leaf_node[6], vertex_list[10], color_list[10]);
	if (pcl::edgeTable[cubeindex] & 2048)
		interpolateEdge (p[3], p[7], leaf_node[3], leaf_node[7], vertex_list[11], color_list[11]);

	// Create the triangle
	for (int i = 0; pcl::triTable[cubeindex][i] != -1; i+=3)
	{
		pcl::PointXYZRGB p1,p2,p3;
		p1.x = vertex_list[pcl::triTable[cubeindex][i  ]][0];
		p1.y = vertex_list[pcl::triTable[cubeindex][i  ]][1];
		p1.z = vertex_list[pcl::triTable[cubeindex][i  ]][2];
		p1.r = color_list [pcl::triTable[cubeindex][i  ]][0];
		p1.g = color_list [pcl::triTable[cubeindex][i  ]][1];
		p1.b = color_list [pcl::triTable[cubeindex][i  ]][2];
		cloud.push_back (p1);
		p2.x = vertex_list[pcl::triTable[cubeindex][i+1]][0];
		p2.y = vertex_list[pcl::triTable[cubeindex][i+1]][1];
		p2.z = vertex_list[pcl::triTable[cubeindex][i+1]][2];
		p2.r = color_list [pcl::triTable[cubeindex][i+1]][0];
		p2.g = color_list [pcl::triTable[cubeindex][i+1]][1];
		p2.b = color_list [pcl::triTable[cubeindex][i+1]][2];
		cloud.push_back (p2);
		p3.x = vertex_list[pcl::triTable[cubeindex][i+2]][0];
		p3.y = vertex_list[pcl::triTable[cubeindex][i+2]][1];
		p3.z = vertex_list[pcl::triTable[cubeindex][i+2]][2];
		p3.r = color_list [pcl::triTable[cubeindex][i+2]][0];
		p3.g = color_list [pcl::triTable[cubeindex][i+2]][1];
		p3.b = color_list [pcl::triTable[cubeindex][i+2]][2];
		cloud.push_back (p3);
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void MarchingCube::performReconstruction(pcl::PolygonMesh &output)
{
	if (!(iso_level_ >= 0 && iso_level_ < 1))
	{
		PCL_ERROR ("[pcl::%s::performReconstruction] Invalid iso level %f! Please use a number between 0 and 1.\n", getClassName ().c_str (), iso_level_);
		output.cloud.width = output.cloud.height = 0;
		output.cloud.data.clear ();
		output.polygons.clear ();
		return;
	}

	// Create grid
	//grid_ = std::vector<float> (res_x_*res_y_*res_z_, 0.0f);
	grid_ = std::vector<float> ( (res_x_+1)*(res_y_+1)*(res_z_+1), 0.0f );

	// Populate tree
	tree_->setInputCloud (input_);

	getBoundingBox ();

	// Transform the point cloud into a voxel grid
	// This needs to be implemented in a child class
	voxelizeData ();



	// Run the actual marching cubes algorithm, store it into a point cloud,
	// and copy the point cloud + connectivity into output
	pcl::PointCloud<pcl::PointXYZRGB> cloud;

	for (int x = 1; x < res_x_-1; ++x)
	{
		for (int y = 1; y < res_y_-1; ++y)
		{
			for (int z = 1; z < res_z_-1; ++z)
			{
				Eigen::Vector3i index_3d (x, y, z);
				std::vector<float> leaf_node;
				getNeighborList1D (leaf_node, index_3d);
				
				int count = 0;
				for (size_t i = 0; i < leaf_node.size(); ++i)
					if ( leaf_node[i]>0.0 && leaf_node[i]<0.5 )
						count++;
					/*if ( leaf_node[i]<=0.0 || leaf_node[i]>=1.0 )
						count++;*/
				//if ( count>=3 )
				if ( count<=0 )
					continue;

				createSurface (leaf_node, index_3d, cloud);
			}
		}
	}
	pcl::toROSMsg (cloud, output.cloud);

	/*pcl::PointCloud<pcl::PointXYZRGB>::Ptr sh_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	*sh_cloud = cloud;
	pcl::visualization::CloudViewer viewer("viewer");
	viewer.showCloud( sh_cloud );
	while ( !viewer.wasStopped() )
	{
	}*/

	output.polygons.resize (cloud.size () / 3);
	for (size_t i = 0; i < output.polygons.size (); ++i)
	{
		pcl::Vertices v;
		v.vertices.resize (3);
		for (int j = 0; j < 3; ++j)
			v.vertices[j] = static_cast<int> (i) * 3 + 2 - j;
			//v.vertices[j] = static_cast<int> (i) * 3 + j;
		output.polygons[i] = v;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void MarchingCube::SetInputTSDFCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tsdf_cloud)
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void MarchingCube::SetMinWeight(float min_w)
{
	mMinWeight = min_w;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void MarchingCube::GetMinWeight(float &min_w)
{
	min_w = mMinWeight;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void MarchingCube::SetTsdfPtr(std::shared_ptr<TSDFBase> tsdf_ptr)
{
	//mTsdfPtr.reset( &tsdf_ptr );
	mTsdfPtr = tsdf_ptr;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void MarchingCube::GetTsdfPtr(std::shared_ptr<TSDFBase> tsdf_ptr)
{
	if ( !mTsdfPtr )
		return;

	//tsdf_ptr.reset( &mTsdfPtr );
	tsdf_ptr = mTsdfPtr;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

