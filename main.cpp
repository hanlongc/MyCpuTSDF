#include <iostream>

#include "application.h"

int main(int argc, char** argv)
{
	std::cout << std::endl
		<< "*************************************************" << std::endl
		<< "* The program use corresponding depth image and *" << std::endl
		<< "* rgb image with known camera pose as input.    *" << std::endl
		<< "* And use construct tsdf point cloud, then use  *" << std::endl
		<< "* marching cube to get mesh.                    *" << std::endl
		<< "*************************************************" << std::endl;

	Application app(argc, argv);
	app.Exec();
	//app.Run();
	//app.TestTexture();
	//app.Texture();

	system("pause");

	return 0;
}