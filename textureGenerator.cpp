#include "textureGenerator.h"

///////////////////////////////////////////////////////////////////////////////////////////////////

TextureGenerator::TextureGenerator()
	: mCResX( 0 )
	, mCResY( 0 )
	, mCResZ( 0 )
	, mGResX( 0 )
	, mGResY( 0 )
	, mGResZ( 0 )
	, mCSizeX( 0.0 )
	, mCSizeY( 0.0 )
	, mCSizeZ( 0.0 )
	, mGSizeX( 0.0 )
	, mGSizeY( 0.0 )
	, mGSizeZ( 0.0 )
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

TextureGenerator::~TextureGenerator()
{
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TextureGenerator::Generate(pcl::PointCloud<pcl::PointXYZRGB>::Ptr mesh_cloud, std::shared_ptr<TSDFBase> tsdf_ptr)
{
	/// 1. Load mesh cloud and color cloud.


	/// 2. Generate textures.

	/// 3. Calculate coordinate and save.
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TextureGenerator::Generate()
{
	/// 1. Load mesh cloud and color cloud.
	std::cout<<"Start generating textures......"<<std::endl<<std::endl;
	std::cout<<"Loading mesh cloud and color cloud......"<<std::endl;

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr meshCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if ( pcl::io::loadPLYFile("my_mesh_for_texture.ply", *meshCloud)<0 )
	{
		std::cerr<<"Error: cannot read the file <my_mesh_for_texture.ply>"<<std::endl;
		return;
	}//end if

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	if ( pcl::io::loadPCDFile("colorCloud.pcd", *colorCloud)<0 )
	{
		std::cerr<<"Error: cannot load the file <colorCloud.pcd>"<<std::endl;
		return;
	}//end if

	std::cout<<"Load mesh cloud and color cloud, done."<<std::endl<<std::endl;

	/// 2. Generate the textures.
	std::cout<<"Generating the textures......"<<std::endl;

	cv::Mat texture;
	std::vector<cv::Point2f> texCoords;
	GenerateTexture(meshCloud, colorCloud, texture, texCoords);

	// Check if valid data.
	if ( texture.empty() )
	{
		std::cout<<"No texture generate."<<std::endl;
		return;
	}//end if

	if ( texCoords.size()<=0 )
	{
		std::cout<<"No texture coordinates generate."<<std::endl;
		return;
	}//end if

	std::cout<<"Generate the texture, done."<<std::endl<<std::endl;

	/// 3. Save.
	std::cout<<"Saving the mesh and texture......"<<std::endl;

	Save(meshCloud, texture, texCoords);

	std::cout<<"Save the mesh and texture, done."<<std::endl<<std::endl;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TextureGenerator::GenerateTexture(pcl::PointCloud<pcl::PointXYZRGB>::Ptr mesh_cloud, 
									   pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud, 
									   cv::Mat &texture, 
									   std::vector<cv::Point2f> &tex_coords)
{
	// For each triangle in the mesh, calculate the texture from the color volume.
	if ( mCResX==0 || mCSizeX<=0.0 )
	{
		std::cout<<"The resolution and size of color must be set."<<std::endl;
		return;
	}//end if

	float pixelSize = mCSizeX / mCResX;
	const int TEX_ROW = 480;
	const int TEX_COL = 600;
	cv::Mat myTexture(TEX_ROW, TEX_COL, CV_8UC4, cv::Scalar::all( 255 ));
	int currRow = 0;
	int currCol = 0;
	int maxCol = 0;

	std::vector<cv::Mat> myTextures;
	std::vector<int> pointIdx;
	std::vector<cv::Point2i> coordInMat;
	
	//if ( mGResX==0 || mGSizeX<=0.0 )
	//{
	//	std::cout<<"The resolution and size of geometry must be set."<<std::endl;
	//	return;
	//}//end if

	/*float stepCX = mCSizeX / mCResX;
	float stepCY = mCSizeY / mCResY;
	float stepCZ = mCSizeZ / mCResZ;*/

	pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
	tree->setInputCloud( color_cloud );

	for (size_t i = 0; i < mesh_cloud->points.size(); i += 3)
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

		if ( (currRow + h)>TEX_ROW )
		{
			currCol += maxCol;
			currRow = 0;
			maxCol = 0;
		}//end if
		if ( (currCol + w)>TEX_COL )
		{
			myTextures.push_back( myTexture );
			pointIdx.push_back( i );
			myTexture.setTo( 255 );
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
				pcl::PointXYZRGB pos;
				pos.x = pt1.x + r * unitVec13.x + c * unitVec12.x;
				pos.y = pt1.y + r * unitVec13.y + c * unitVec12.y;
				pos.z = pt1.z + r * unitVec13.z + c * unitVec12.z;

				std::vector<int> nn_indices;
				std::vector<float> nn_sqrt_dists;

				tree->nearestKSearchT(pos, 1, nn_indices, nn_sqrt_dists);

				myTexture.at<cv::Vec4b>(currRow, currCol)[0] = color_cloud->points[ nn_indices[0] ].b;
				myTexture.at<cv::Vec4b>(currRow, currCol)[1] = color_cloud->points[ nn_indices[0] ].g;
				myTexture.at<cv::Vec4b>(currRow, currCol)[2] = color_cloud->points[ nn_indices[0] ].r;

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

	myTextures.push_back( myTexture );
	pointIdx.push_back( mesh_cloud->points.size()-1 );

	if ( myTextures.size()<1 )
	{
		std::cerr<<"Error: not texture generated."<<std::endl;
		return;
	}//end if

	if ( myTextures.size()>1 )
	{
		cv::Mat finalTexture;

		StitchAndUpdate(finalTexture, coordInMat, myTextures, currCol, pointIdx);

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

			tex_coords.push_back( texPt );
		}//end for

		cv::imwrite("finalTexture.png", finalTexture);

		if ( texture.empty() )
			texture.create(finalTexture.rows, finalTexture.cols, finalTexture.type());
		finalTexture.copyTo( texture );

		for (int i=0; i<myTextures.size(); ++i)
		{
			std::ostringstream ostr;
			ostr << i;
			std::string texName = "textures" + ostr.str() + ".png";
			cv::imwrite(texName, myTextures[i]);
		}//end for
	}//end if
	else if ( myTextures.size()==1 )
	{
		int textureRow = myTextures[0].rows;
		int textureCol = myTextures[0].cols;

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

			tex_coords.push_back( texPt );
		}//end for

		cv::imwrite("finalTexture.png", myTextures[0]);

		if ( texture.empty() )
			texture.create(myTextures[0].rows, myTextures[0].cols, myTextures[0].type());
		myTextures[0].copyTo( texture );
	}//end else if
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TextureGenerator::Save(pcl::PointCloud<pcl::PointXYZRGB>::Ptr mesh_cloud, 
							cv::Mat &texture, 
							std::vector<cv::Point2f> &tex_coords)
{
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

	for (int i=0; i<tex_coords.size(); i+=3)
	{
		outfile<<3<<" "<<(i+2)<<" "<<(i+1)<<" "<<i<<" "<<6<<" "<<std::setprecision(5)<<tex_coords[i+2].x<<" "<<tex_coords[i+2].y<<" ";
		outfile<<std::setprecision(5)<<tex_coords[i+1].x<<" "<<tex_coords[i+1].y<<" "<<tex_coords[i].x<<" "<<tex_coords[i].y<<std::endl;
	}//end for

	outfile.close();
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TextureGenerator::SetColorResolution(int res_x, int res_y, int res_z)
{
	mCResX = res_x;
	mCResY = res_y;
	mCResZ = res_z;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TextureGenerator::SetColorSize(float size_x, float size_y, float size_z)
{
	mCSizeX = size_x;
	mCSizeY = size_y;
	mCSizeZ = size_z;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TextureGenerator::SetGeometrySize(float size_x, float size_y, float size_z)
{
	mGSizeX = size_x;
	mGSizeY = size_y;
	mGSizeZ = size_z;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TextureGenerator::SetGeometryResolution(int res_x, int res_y, int res_z)
{
	mGResX = res_x;
	mGResY = res_y;
	mGResZ = res_z;
}

///////////////////////////////////////////////////////////////////////////////////////////////////

void TextureGenerator::StitchAndUpdate(cv::Mat &texture, 
									   std::vector<cv::Point2i> &coords, 
									   std::vector<cv::Mat> &textures, 
									   int curr_col, 
									   std::vector<int> &point_idx)
{
	int textureRow = textures[0].rows;
	int textureCol = (textures.size() -1) * textures[0].cols + curr_col;

	texture.create(textureRow, textureCol, CV_8UC4);
	texture.setTo( 255 );

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
			coords[n].x += ((i+1)*originCol);
		}//end for
	}//end for
}

///////////////////////////////////////////////////////////////////////////////////////////////////

