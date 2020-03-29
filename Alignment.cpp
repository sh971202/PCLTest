#include <iostream>
#include <fstream>
#include <string>
#include <time.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/common/common.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

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
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_color(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_class(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_class_after(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_color_after(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	t = clock();
	pcl::io::loadPLYFile(argv[1], *cloud_color);
	pcl::io::loadPLYFile(argv[2], *cloud_class);
	cout << "Load point cloud done!!" << " Time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;

	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.addCoordinateSystem(1, "cloud", 0);
	viewer.initCameraParameters();
	viewer.setBackgroundColor(0, 0, 0);

	//cout << "Color Size: " << cloud_color->points[12345] << endl;
	//cout << "Class Size: " << cloud_class->points[12345] << endl;


	
	pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
	std::vector<int> pointIdxNKNSearch(1);
	std::vector<float> pointNKNSquaredDistance(1);

	kdtree.setInputCloud(cloud_class);

	for (int i = 0; i < cloud_color->size(); i++)
	{
		if (kdtree.nearestKSearch(cloud_color->points[i], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			if (pointNKNSquaredDistance[0] < 0.0000001) 
			{
				cloud_color_after->points.push_back(cloud_color->points[i]);
				cloud_class_after->points.push_back(cloud_class->points[pointIdxNKNSearch[0]]);
			}
		}
	}

	cout << "Color Size: " << cloud_color_after->points.size() << endl;
	cout << "Class Size: " << cloud_class_after->points.size() << endl;

	viewer.addPointCloud<pcl::PointXYZRGBNormal>(cloud_color_after, "scene");
	viewer.addPointCloud<pcl::PointXYZRGBNormal>(cloud_class_after, "scene2");
	
	pcl::PLYWriter writer;

	writer.write("output.ply", *cloud_color_after, true, false);
	writer.write("output2.ply", *cloud_class_after, true, false);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return 0;
}