#include <iostream>
#include <fstream>
#include <string>
#include <time.h>

#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>

#include "pre_processing.h"
#include "registration.h"

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkRenderingOpenGL);

using namespace std;

int main(int argc, char *argv[])
{
	clock_t t;
	t = clock();

	string file_name = argv[1];
	//load model
	pcl::PointCloud<pcl::PointNormal>::Ptr sample_cloud_normal(new pcl::PointCloud<pcl::PointNormal>);
	pcl::io::loadPLYFile(file_name, *sample_cloud_normal);

	//adjusting by scale
	/*Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
	scale(0, 0) = atof(argv[2]);
	scale(1, 1) = atof(argv[3]);
	scale(2, 2) = atof(argv[4]);

	pcl::transformPointCloud(*sample_cloud_normal, *sample_cloud_normal, scale);
	*/
	//------------------Downsample-------------------
	pcl::console::print_highlight("Downsampling...\n");
	pcl::VoxelGrid<pcl::PointNormal> grid;
	const float leaf = 0.03f;
	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(sample_cloud_normal);
	grid.filter(*sample_cloud_normal);
	cout << "Model points:" << sample_cloud_normal->size() << endl;
	//------------------Downsample-------------------

	for (int i = 0; i < sample_cloud_normal->points.size(); i++)
	{
		sample_cloud_normal->points[i].curvature = 0;
	}

	//write point cloud into PLY file formate
	pcl::PLYWriter writer;
	file_name = file_name.assign(file_name,0,file_name.find(".ply"));
	writer.write(file_name + ".ply", *sample_cloud_normal, true, false);

	cout << "Modification done!!" << " Time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;

	//------------------result with visualization (optional)
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.initCameraParameters();
	viewer.setBackgroundColor(0, 0, 0);
	viewer.addCoordinateSystem(3, "cloud", 0);

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color(sample_cloud_normal, 0, 255, 0);
	viewer.addPointCloud<pcl::PointNormal>(sample_cloud_normal, single_color,"M");

	viewer.addPointCloudNormals<pcl::PointNormal>(sample_cloud_normal, 1, 0.02, "normal");


	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	//---------------------------
	return 0;
}