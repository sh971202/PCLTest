#include <iostream>
#include <fstream>
#include <string>
#include <time.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/features/esf.h>

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkRenderingOpenGL);

using namespace std;

int main(int arbc,char *argv[])
{
	pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esf;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::ESFSignature640>::Ptr model1_descriptor(new pcl::PointCloud<pcl::ESFSignature640>);

	pcl::io::loadPLYFile(argv[1], *cloud);
	// ESF estimation object.
	esf.setInputCloud(cloud);

	esf.compute(*model1_descriptor);

	pcl::PointCloud<pcl::PointXYZ> Model[12];

	pcl::io::loadPLYFile("./Model_pointcloud(modify)/chair/chair1.ply", Model[0]);
	pcl::io::loadPLYFile("./Model_pointcloud(modify)/chair/chair2.ply", Model[1]);
	pcl::io::loadPLYFile("./Model_pointcloud(modify)/chair/chair3.ply", Model[2]);
	pcl::io::loadPLYFile("./Model_pointcloud(modify)/chair/chair4.ply", Model[3]);
	pcl::io::loadPLYFile("./Model_pointcloud(modify)/sofa/sofa1.ply", Model[4]);
	pcl::io::loadPLYFile("./Model_pointcloud(modify)/sofa/sofa2.ply", Model[5]);
	pcl::io::loadPLYFile("./Model_pointcloud(modify)/sofa/sofa3.ply", Model[6]);
	pcl::io::loadPLYFile("./Model_pointcloud(modify)/sofa/sofa4.ply", Model[7]);
	pcl::io::loadPLYFile("./Model_pointcloud(modify)/table/table1.ply", Model[8]);
	pcl::io::loadPLYFile("./Model_pointcloud(modify)/table/table2.ply", Model[9]);
	pcl::io::loadPLYFile("./Model_pointcloud(modify)/table/table3.ply", Model[10]);
	pcl::io::loadPLYFile("./Model_pointcloud(modify)/table/table4.ply", Model[11]);

	for (int k = 0; k < 12; k++)
	{
		clock_t t;

		pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);

		*temp = Model[k];

		// Object for storing the ESF descriptor.
		pcl::PointCloud<pcl::ESFSignature640>::Ptr model2_descriptor(new pcl::PointCloud<pcl::ESFSignature640>);

		t = clock();
		// ESF estimation object. 
		esf.setInputCloud(temp);

		esf.compute(*model2_descriptor);

		float dis = 0.0;
		for (int i = 0; i < model1_descriptor->points[0].descriptorSize(); i++)
		{
			float a = model1_descriptor->points[0].histogram[i];
			float b = model2_descriptor->points[0].histogram[i];
			dis += (a - b)*(a - b);
		}
		dis = sqrt(dis);

		cout << "ESF distance: " << dis << endl;
		cout << "ESF Time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;
	}

	system("pause");
	return 0;
}