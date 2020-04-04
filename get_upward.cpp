#include <iostream>
#include <fstream>
#include <string>
#include <time.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>


#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include "pre_processing.h"
#include "registration.h"
#include "save_model.h"
#include "./json/save_json.h"

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkRenderingOpenGL);

using namespace std;
pre_processing process;
save_json test;

void save_data(string filename)
{
	ofstream fout;

	fout.open(filename + ".txt");

	fout << "Coefficients of Ground plane" << endl;
	pcl::ModelCoefficients coefficients = process.get_plane_coefficients();
	fout << coefficients.values[0] << " " << coefficients.values[1] << " " << coefficients.values[2] << " " << coefficients.values[3] << endl;

	fout << "Mass position of Ground plane" << endl;
	Eigen::Vector4f centroid = process.get_plane_centroid();
	fout << centroid[0] << " " << centroid[1] << " " << centroid[2] << endl;

	fout << "Center of Ground plane" << endl;
	pcl::PointXYZ plane_center = process.get_plane_center();
	fout << plane_center.x << " " << plane_center.y << " " << plane_center.z << endl;

	fout << "Size of Ground plane" << endl;
	pcl::PointXYZ plane_size = process.get_plane_size();
	fout << plane_size.x << " " << plane_size.y << " " << plane_size.z << endl;
	/*
	fout << "Euler angle" << endl;
	fout << roll / M_PI * 180 << " " << pitch / M_PI * 180 << " " << yaw / M_PI * 180 << endl;
	*/
	int wall_number = process.get_walls_number();

	fout << "Wall segmentation" << endl;
	fout << wall_number << endl;

	for (int i = 0; i < wall_number; i++)
	{
		pcl::PointXYZ wall_center, wall_coefficients, wall_size;

		wall_coefficients = process.get_walls_coefficients(i);
		wall_center = process.get_walls_center(i);
		wall_size = process.get_walls_size(i);

		fout << wall_coefficients.x << " " << wall_coefficients.y << " " << wall_coefficients.z << endl;
		fout << wall_center.x << " " << wall_center.y << " " << wall_center.z << endl;
		fout << wall_size.x << " " << wall_size.y << " " << wall_size.z << endl;
	}

}

void save_data_json()
{
	test.save_ground_data(process.get_plane_coefficients().values.data(), process.get_plane_centroid().data(), process.get_plane_center().data, process.get_plane_size().data);
	for (int i = 0; i < process.get_walls_number(); i++)
	{
		test.save_wall_data(process.get_walls_coefficients(i).data, process.get_walls_center(i).data, process.get_walls_size(i).data);
	}
}

void load_data(string filename)
{
	float temp;
	ifstream fin;
	string line;

	fin.open(filename);

	if (!fin.is_open())
	{
		cout << "Open file error!!";
		return;
	}

	while (getline(fin, line))
	{
		cout << line << endl;

		if (line == "Coefficients of Ground plane")
		{
			pcl::ModelCoefficients coefficients;
			for (int i = 0; i < 4; i++)
			{
				fin >> temp;
				cout << temp << " ";
				coefficients.values.push_back(temp);
			}
			process.set_plane_coefficients(coefficients);
		}
		else if (line == "Mass position of Ground plane")
		{
			Eigen::Vector4f centroid;
			for (int i = 0; i < 3; i++)
			{
				fin >> temp;
				cout << temp << " ";
				centroid[i] = temp;
			}
			process.set_plane_centroid(centroid);
		}
		else if (line == "Center of Ground plane")
		{
			pcl::PointXYZ plane_center;
			fin >> plane_center.x >> plane_center.y >> plane_center.z;
			cout << plane_center.x << " " << plane_center.y << " " << plane_center.z;
			process.set_plane_center(plane_center);
		}
		else if (line == "Size of Ground plane")
		{
			pcl::PointXYZ plane_size;
			fin >> plane_size.x >> plane_size.y >> plane_size.z;
			cout << plane_size.x << " " << plane_size.y << " " << plane_size.z << endl;
			process.set_plane_size(plane_size);
		}
		else if (line == "Wall segmentation")
		{
			int wall_number = 0;
			fin >> wall_number;
			cout << wall_number << endl;
			process.set_walls_number(wall_number);
			for (int i = 0; i < wall_number; i++)
			{
				pcl::PointXYZ wall_center, wall_coefficients, wall_size;

				fin >> wall_coefficients.x >> wall_coefficients.y >> wall_coefficients.z;
				fin >> wall_center.x >> wall_center.y >> wall_center.z;
				fin >> wall_size.x >> wall_size.y >> wall_size.z;

				cout << wall_coefficients.x << " " << wall_coefficients.y << " " << wall_coefficients.z << endl;
				cout << wall_center.x << " " << wall_center.y << " " << wall_center.z << endl;
				cout << wall_size.x << " " << wall_size.y << " " << wall_size.z << endl;

				process.set_walls_coefficients(wall_coefficients, i);
				process.set_walls_center(wall_center, i);
				process.set_walls_size(wall_size, i);
			}
		}

	}

}

void Extract_wall_and_ground(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_class, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr ground, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr wall)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal> temp_wall;
	pcl::PointCloud<pcl::PointXYZRGBNormal> temp_ground;


	clock_t t;

	t = clock();

	for (int n = 0; n < cloud_class->points.size(); n++)
	{
		float r, g, b;

		r = cloud_class->points[n].r;
		g = cloud_class->points[n].g;
		b = cloud_class->points[n].b;


		if (r == 220 && g == 220 && b == 220)
		{
			temp_ground.push_back(cloud_class->points[n]);
		}
		else if (r == 150 && g == 150 && b == 150)
		{
			temp_wall.push_back(cloud_class->points[n]);
		}
	}

	pcl::console::print_highlight("Downsampling...\n");
	pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid;
	float leaf = 0.03f;
	grid.setLeafSize(leaf, leaf, leaf);

	grid.setInputCloud(temp_wall.makeShared());
	grid.filter(temp_wall);


	grid.setInputCloud(temp_ground.makeShared());
	grid.filter(temp_ground);

	*wall = temp_wall;
	*ground = temp_ground;

	cout << "Extract_wall_and_ground done!!" << " Time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;
}

int main(int argc, char *argv[])
{
	clock_t t;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_color(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr wall_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	t = clock();
	pcl::io::loadPLYFile("./lab2/123_class.ply", *cloud_color);

	cout << "Load point cloud done!!" << " Time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;

	//load_data(argv[2]);
	//---------------Transform point cloud to upward orientation------------------

	for (int n = 0; n < cloud_color->points.size(); n++)
	{
		if(process.class_mapping(cloud_color->points[n]) == 4)
			plane_cloud->push_back(cloud_color->points[n]);
	}

	process.processing_ground_plane(plane_cloud);
	process.compute_ground_normal_transformation();
	pcl::transformPointCloudWithNormals(*cloud_color, *cloud_color, process.get_plane_transformation());  //cloud with normal
	//----------------------------------------------------------------------------
	//---------------Transform point cloud to forward orientation-------------
	for (int n = 0; n < cloud_color->points.size(); n++)
	{
		if (process.class_mapping(cloud_color->points[n]) == 11)
			wall_cloud->push_back(cloud_color->points[n]);
	}

	process.processing_wall(wall_cloud);
	process.compute_wall_normal_transformation();
	pcl::transformPointCloudWithNormals(*cloud_color, *cloud_color, process.get_wall_transformation());  //cloud with normal
	//----------------------------------------------------------------------------
	/*
	pcl::PointCloud<pcl::PointXYZ> cloud_copy;

	pcl::copyPointCloud(*cloud_color, cloud_copy); //copy original cloud

	pcl::PointXYZ minpt, maxpt, plane_size;
	pcl::getMinMax3D(cloud_copy, minpt, maxpt);

	pcl::PointXYZ pt1(minpt.x, minpt.y, minpt.z);
	pcl::PointXYZ pt2(minpt.x, minpt.y, maxpt.z);
	pcl::PointXYZ pt3(maxpt.x, minpt.y, maxpt.z);
	pcl::PointXYZ pt4(maxpt.x, minpt.y, minpt.z);
	pcl::PointXYZ pt5(minpt.x, maxpt.y, minpt.z);
	pcl::PointXYZ pt6(minpt.x, maxpt.y, maxpt.z);
	pcl::PointXYZ pt7(maxpt.x, maxpt.y, maxpt.z);
	pcl::PointXYZ pt8(maxpt.x, maxpt.y, minpt.z);

	pcl::PointXYZ box_center;

	box_center.x = (pt1.x + pt2.x + pt3.x + pt4.x + pt5.x + pt6.x + pt7.x + pt8.x) / 8;  // x position of bounding box 
	box_center.y = 0;
	box_center.z = (pt1.z + pt2.z + pt3.z + pt4.z + pt5.z + pt6.z + pt7.z + pt8.z) / 8;  // z position of bounding box 


	cout << "Max-Min(x,y,z): " << maxpt.x - minpt.x << "," << maxpt.y - minpt.y << "," << maxpt.z - minpt.z << endl;
	cout << "box_center(x,y,z): " << box_center.x << "," << box_center.y << "," << box_center.z << endl;

	//save plane model data
	plane_size.x = maxpt.x - minpt.x;
	plane_size.y = 0.0001;
	plane_size.z = maxpt.z - minpt.z;

	process.set_plane_size(plane_size);
	process.set_plane_center(box_center);
	
	save_data("model_information");
	*/
	
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.addCoordinateSystem(1, "cloud", 0);
	viewer.initCameraParameters();
	viewer.setBackgroundColor(0, 0, 0);

	//second step
	
	viewer.addPointCloud<pcl::PointXYZRGBNormal>(cloud_color,"scene");
	
	pcl::PLYWriter writer;
	
	writer.write("output_class.ply", *cloud_color, true, false);
	
	save_data_json();
	test.save_obj_data("non", 0, process.get_wall_transformation().data());
	test.save_file("output");
	
	/*
	//first step
	for (int i = 0; i < process.get_walls_number(); i++)
	{
		pcl::PointXYZ end(process.get_walls_center(i).x +process.get_walls_coefficients(i).x, process.get_walls_center(i).y + process.get_walls_coefficients(i).y, process.get_walls_center(i).z + process.get_walls_coefficients(i).z);
		viewer.addArrow(end, process.get_walls_center(i), 0.0, 0.0, 1.0, false, "arrow_z1" +to_string(i));
	}
	
	viewer.addPointCloud(process.get_plane_cloud().makeShared(),"ground");
	
	for (int i = 0; i < process.get_walls_number(); i++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);

		*temp = process.get_walls_cloud(i);

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tc_handler(temp, 100, i*60, 0);
		viewer.addPointCloud(temp, tc_handler,"w" + to_string(i));
	}
	*/
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return 0;
}