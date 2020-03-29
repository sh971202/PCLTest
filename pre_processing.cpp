#include <iostream>
#include <vector>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include "pre_processing.h"

using namespace std;

void pre_processing::processing_ground_plane(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
	clock_t t;
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);

	t = clock();
	
	pcl::copyPointCloud(*cloud, *cloud_filtered); //copy original cloud
	
	/*
	pcl::console::print_highlight("Downsampling...\n");
	pcl::VoxelGrid<pcl::PointXYZ> grid;
	const float leaf = 0.02f;
	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(cloud_filtered);
	grid.filter(*cloud_filtered);
	cout << "Scene points:" << cloud_filtered->size() << endl;
	*/
	

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.02);

	seg.setInputCloud(cloud_filtered);
	seg.segment(*inliers, plane_coefficients);

	//execution time
	cout << "Plane segmentation time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;
	//plane normal vector : ax+by+cz+d=0 => (a,b,c,d)
	cout << "Model Coefficients: " << plane_coefficients.values[0] << " " << plane_coefficients.values[1] << " " << plane_coefficients.values[2] << " " << plane_coefficients.values[3] << " " << endl;
	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;

	// Extract the inliers
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers);
	extract.setNegative(false);
	extract.filter(plane_cloud);

	//compute point cloud mass
	pcl::compute3DCentroid(plane_cloud, plane_centroid);
}

void pre_processing::compute_ground_normal_transformation()
{
	//---------------Transform point cloud to upward orientation-------------
	Eigen::Vector3f normal;

	normal[0] = plane_coefficients.values[0];
	normal[1] = plane_coefficients.values[1];
	normal[2] = plane_coefficients.values[2];

	Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();


	// transform plane normal to Y-axis

	transform_1 = CreateRotateMatrix(normal, Eigen::Vector3f::UnitY());
	//pcl::transformPointCloud(*cloud, *cloud, transform_1);

	Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Identity();

	//compute point cloud mass
	if (plane_cloud.size() == 0)
	{
		cout << "Read txt file." << endl;
	}
	else
	{
		pcl::transformPointCloud(plane_cloud, plane_cloud, transform_1);
		pcl::compute3DCentroid(plane_cloud, plane_centroid);
			cout << "Point cloud mass : ("
				<< plane_centroid[0] << ", "
				<< plane_centroid[1] << ", "
				<< plane_centroid[2] << ")." << endl;
	}

	transform_2(0, 3) = -plane_centroid[0];
	transform_2(1, 3) = -plane_centroid[1];
	transform_2(2, 3) = -plane_centroid[2];

	pcl::transformPointCloud(plane_cloud, plane_cloud, transform_2);

	//pcl::transformPointCloud(*cloud, *cloud, transform_2);
	plane_transformation = transform_2 * transform_1;
}

pcl::PointCloud<pcl::PointXYZ> pre_processing::get_plane_cloud()
{
	return plane_cloud;
}

Eigen::Vector4f pre_processing::get_plane_centroid()
{
	return plane_centroid;
}

pcl::ModelCoefficients pre_processing::get_plane_coefficients()
{
	return plane_coefficients;
}

pcl::PointXYZ pre_processing::get_plane_center()
{
	return plane_center;
}

pcl::PointXYZ pre_processing::get_plane_size()
{
	return plane_size;
}

Eigen::Matrix4f pre_processing::get_plane_transformation()
{
	return plane_transformation;
}

void pre_processing::set_plane_centroid(Eigen::Vector4f centroid)
{
	plane_centroid = centroid;
}

void pre_processing::set_plane_coefficients(pcl::ModelCoefficients coefficients)
{
	plane_coefficients = coefficients;
}

void pre_processing::set_plane_center(pcl::PointXYZ center)
{
	plane_center = center;
}

void pre_processing::set_plane_size(pcl::PointXYZ size)
{
	plane_size = size;
}

void pre_processing::processing_wall(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
	clock_t t;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>),  cloud_f(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	Eigen::Vector4f centroid;
	t = clock();

	pcl::copyPointCloud(*cloud, *cloud_filtered); //copy original cloud
	
	
	pcl::console::print_highlight("Downsampling...\n");
	pcl::VoxelGrid<pcl::PointXYZ> grid;
	const float leaf = 0.02f;
	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(cloud_filtered);
	grid.filter(*cloud_filtered);
	cout << "Scene points:" << cloud_filtered->size() << endl;
	
	
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	//seg.setMaxIterations(1000);
	seg.setDistanceThreshold(0.025);

	pcl::ExtractIndices<pcl::PointXYZ> extract;

	int nr_points = (int)cloud_filtered->points.size();
	wall_number = 0;
	// While 10% of the original cloud is still there and haven't reached four walls 
	while (cloud_filtered->points.size() > 0.1 * nr_points && wall_number != 3)
	{
		t = clock();
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud_filtered);
		seg.segment(*inliers, *coefficients);
		cout << "Wall segmentation time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;
		if (inliers->indices.size() < 5000)
		{
			extract.setInputCloud(cloud_filtered);
			extract.setIndices(inliers);
			extract.setNegative(true);
			extract.filter(*cloud_f);
			cloud_filtered.swap(cloud_f);
			cout << "Could not estimate a planar model for the given dataset." << endl;
			cout << "Point cloud Size: " << inliers->indices.size() << endl << endl;
			continue;
		}

		// Extract the inliers
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(walls_cloud[wall_number]);
		cout << "PointCloud representing the planar component: " << walls_cloud[wall_number].width * walls_cloud[wall_number].height << " data points." << endl;
		cout << "Model Coefficients: " << coefficients->values[0] << " " << coefficients->values[1] << " " << coefficients->values[2] << " " << coefficients->values[3] << " " << endl;

		//normal vector of the walls
		wall_coefficients[wall_number].x = coefficients->values[0];
		wall_coefficients[wall_number].y = coefficients->values[1];
		wall_coefficients[wall_number].z = coefficients->values[2];

		// Create the filtering object
		extract.setNegative(true);
		extract.filter(*cloud_f);
		cloud_filtered.swap(cloud_f);
		wall_number++;

		cout << endl;
	}

}

void pre_processing::compute_wall_normal_transformation()
{
	if (wall_number == 0)
	{
		cout << "These is no wall to define Z-axis" << endl;
		return;
	}

	//---------------Transform point cloud to forward orientation-------------
	Eigen::Vector3f normal;

	//using normal vector of the first wall to define Z-axis 
	normal[0] = wall_coefficients[0].x;
	normal[1] = 0;
	normal[2] = wall_coefficients[0].z;

	wall_transformation = Eigen::Matrix4f::Identity();
	 
	wall_transformation = CreateRotateMatrix(normal, Eigen::Vector3f::UnitZ());
	
	//----------------------------------------------------------------------------

	if (walls_cloud[0].size() == 0)
	{
		cout << "Read txt file." << endl;
		return;
	}

	//-----------------Set walls and plane point cloud size--------------------------------------

	for (int i = 1; i < wall_number; i++)
	{
		Eigen::Vector4f transformed_normal;

		transformed_normal[0] = wall_coefficients[i].x;
		transformed_normal[1] = wall_coefficients[i].y;
		transformed_normal[2] = wall_coefficients[i].z;
		transformed_normal[3] = 0;

		transformed_normal = wall_transformation * transformed_normal;

		wall_coefficients[i].x = transformed_normal[0];
		wall_coefficients[i].y = transformed_normal[1];
		wall_coefficients[i].z = transformed_normal[2];
	}

	for (int i=0; i < wall_number; i++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
		
		pcl::transformPointCloud(walls_cloud[i], walls_cloud[i], wall_transformation);

		pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
		// build the filter
		outrem.setInputCloud(walls_cloud[i].makeShared());
		outrem.setRadiusSearch(0.5);
		outrem.setMinNeighborsInRadius(50);
		
		// apply filter
		outrem.filter(walls_cloud[i]);

		pcl::PointXYZ minpt, maxpt;

		if (i != 0)
		{
			Eigen::Vector3f normal;

			normal[0] = wall_coefficients[i].x;
			normal[1] = 0;
			normal[2] = wall_coefficients[i].z;

			Eigen::Matrix4f temp_transformation = Eigen::Matrix4f::Identity();

			temp_transformation = CreateRotateMatrix(normal, Eigen::Vector3f::UnitZ());

			pcl::transformPointCloud(walls_cloud[i], *temp, temp_transformation);

			pcl::getMinMax3D(*temp, minpt, maxpt);

			wall_size[i].x = maxpt.x - minpt.x;
			//wall_size[i].y = maxpt.y - minpt.y;
			wall_size[i].z = maxpt.z - minpt.z;

			cout << "Max-Min(x,y,z): " << maxpt.x - minpt.x << "," << maxpt.y - minpt.y << "," << maxpt.z - minpt.z << endl;

			Eigen::Vector4f temp_minpt,temp_maxpt;

			temp_minpt[0] = minpt.x;
			temp_minpt[1] = minpt.y;
			temp_minpt[2] = minpt.z;
			temp_minpt[3] = 0;

			temp_minpt = temp_transformation.inverse() * temp_minpt;

			minpt.x = temp_minpt[0];
			minpt.y = temp_minpt[1];
			minpt.z = temp_minpt[2];

			temp_maxpt[0] = maxpt.x;
			temp_maxpt[1] = maxpt.y;
			temp_maxpt[2] = maxpt.z;
			temp_maxpt[3] = 0;

			temp_maxpt = temp_transformation.inverse() * temp_maxpt;

			maxpt.x = temp_maxpt[0];
			maxpt.y = temp_maxpt[1];
			maxpt.z = temp_maxpt[2];

			pcl::PointXYZ pt1(minpt.x, minpt.y, minpt.z);
			pcl::PointXYZ pt2(minpt.x, minpt.y, maxpt.z);
			pcl::PointXYZ pt3(maxpt.x, minpt.y, maxpt.z);
			pcl::PointXYZ pt4(maxpt.x, minpt.y, minpt.z);
			pcl::PointXYZ pt5(minpt.x, maxpt.y, minpt.z);
			pcl::PointXYZ pt6(minpt.x, maxpt.y, maxpt.z);
			pcl::PointXYZ pt7(maxpt.x, maxpt.y, maxpt.z);
			pcl::PointXYZ pt8(maxpt.x, maxpt.y, minpt.z);

			/*
			viewer.addLine(pt1, pt2, 1.0, 0.0, 0.0, id + "1 edge");
			viewer.addLine(pt1, pt4, 1.0, 0.0, 0.0, id + "2 edge");
			viewer.addLine(pt1, pt5, 1.0, 0.0, 0.0, id + "3 edge");
			viewer.addLine(pt5, pt6, 1.0, 0.0, 0.0, id + "4 edge");
			viewer.addLine(pt5, pt8, 1.0, 0.0, 0.0, id + "5 edge");
			viewer.addLine(pt2, pt6, 1.0, 0.0, 0.0, id + "6 edge");
			viewer.addLine(pt6, pt7, 1.0, 0.0, 0.0, id + "7 edge");
			viewer.addLine(pt7, pt8, 1.0, 0.0, 0.0, id + "8 edge");
			viewer.addLine(pt2, pt3, 1.0, 0.0, 0.0, id + "9 edge");
			viewer.addLine(pt4, pt8, 1.0, 0.0, 0.0, id + "10 edge");
			viewer.addLine(pt3, pt4, 1.0, 0.0, 0.0, id + "11 edge");
			viewer.addLine(pt3, pt7, 1.0, 0.0, 0.0, id + "12 edge");
			*/
			pcl::PointXYZ box_center;

			box_center.x = (pt1.x + pt2.x + pt3.x + pt4.x + pt5.x + pt6.x + pt7.x + pt8.x) / 8;  // x position of bounding box 
			box_center.y = (pt1.y + pt2.y + pt3.y + pt4.y + pt5.y + pt6.y + pt7.y + pt8.y) / 8;  // y position of bounding box 
			box_center.z = (pt1.z + pt2.z + pt3.z + pt4.z + pt5.z + pt6.z + pt7.z + pt8.z) / 8;  // z position of bounding box 

			cout << "box_center(x,y,z): " << box_center.x << "," << box_center.y << "," << box_center.z << endl;

			//save wall model data
			wall_center[i] = box_center;
			wall_size[i].y = box_center.y *2;

		}
		else
		{
			pcl::getMinMax3D(walls_cloud[i], minpt, maxpt);

			pcl::PointXYZ pt1(minpt.x, minpt.y, minpt.z);
			pcl::PointXYZ pt2(minpt.x, minpt.y, maxpt.z);
			pcl::PointXYZ pt3(maxpt.x, minpt.y, maxpt.z);
			pcl::PointXYZ pt4(maxpt.x, minpt.y, minpt.z);
			pcl::PointXYZ pt5(minpt.x, maxpt.y, minpt.z);
			pcl::PointXYZ pt6(minpt.x, maxpt.y, maxpt.z);
			pcl::PointXYZ pt7(maxpt.x, maxpt.y, maxpt.z);
			pcl::PointXYZ pt8(maxpt.x, maxpt.y, minpt.z);

			/*
			viewer.addLine(pt1, pt2, 1.0, 0.0, 0.0, id + "1 edge");
			viewer.addLine(pt1, pt4, 1.0, 0.0, 0.0, id + "2 edge");
			viewer.addLine(pt1, pt5, 1.0, 0.0, 0.0, id + "3 edge");
			viewer.addLine(pt5, pt6, 1.0, 0.0, 0.0, id + "4 edge");
			viewer.addLine(pt5, pt8, 1.0, 0.0, 0.0, id + "5 edge");
			viewer.addLine(pt2, pt6, 1.0, 0.0, 0.0, id + "6 edge");
			viewer.addLine(pt6, pt7, 1.0, 0.0, 0.0, id + "7 edge");
			viewer.addLine(pt7, pt8, 1.0, 0.0, 0.0, id + "8 edge");
			viewer.addLine(pt2, pt3, 1.0, 0.0, 0.0, id + "9 edge");
			viewer.addLine(pt4, pt8, 1.0, 0.0, 0.0, id + "10 edge");
			viewer.addLine(pt3, pt4, 1.0, 0.0, 0.0, id + "11 edge");
			viewer.addLine(pt3, pt7, 1.0, 0.0, 0.0, id + "12 edge");
			*/
			pcl::PointXYZ box_center;

			box_center.x = (pt1.x + pt2.x + pt3.x + pt4.x + pt5.x + pt6.x + pt7.x + pt8.x) / 8;  // x position of bounding box 
			box_center.y = (pt1.y + pt2.y + pt3.y + pt4.y + pt5.y + pt6.y + pt7.y + pt8.y) / 8;  // y position of bounding box 
			box_center.z = (pt1.z + pt2.z + pt3.z + pt4.z + pt5.z + pt6.z + pt7.z + pt8.z) / 8;  // z position of bounding box 

			cout << "Max-Min(x,y,z): " << maxpt.x - minpt.x << "," << maxpt.y - minpt.y << "," << maxpt.z - minpt.z << endl;
			cout << "box_center(x,y,z): " << box_center.x << "," << box_center.y << "," << box_center.z << endl;

			//save wall model data
			wall_center[i] = box_center;
			wall_size[i].x = maxpt.x - minpt.x;
			wall_size[i].y = box_center.y *2;
			wall_size[i].z = maxpt.z - minpt.z;
		}
	}
	
	pcl::transformPointCloud(plane_cloud, plane_cloud, wall_transformation);

	pcl::PointXYZ minpt, maxpt;
	pcl::getMinMax3D(plane_cloud, minpt, maxpt);

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
	box_center.y = (pt1.y + pt2.y + pt3.y + pt4.y + pt5.y + pt6.y + pt7.y + pt8.y) / 8;  // y position of bounding box 
	box_center.z = (pt1.z + pt2.z + pt3.z + pt4.z + pt5.z + pt6.z + pt7.z + pt8.z) / 8;  // z position of bounding box 


	cout << "Max-Min(x,y,z): " << maxpt.x - minpt.x << "," << maxpt.y - minpt.y << "," << maxpt.z - minpt.z << endl;
	cout << "box_center(x,y,z): " << box_center.x << "," << box_center.y << "," << box_center.z << endl;

	//save plane model data
	plane_center = box_center;
	plane_size.x = maxpt.x - minpt.x;
	plane_size.y = maxpt.y - minpt.y;
	plane_size.z = maxpt.z - minpt.z;
}

pcl::PointCloud<pcl::PointXYZ> pre_processing::get_walls_cloud(int number)
{
	return walls_cloud[number];
}

pcl::PointXYZ pre_processing::get_walls_center(int number)
{
	return wall_center[number];
}

pcl::PointXYZ pre_processing::get_walls_coefficients(int number)
{
	return wall_coefficients[number];
}

pcl::PointXYZ pre_processing::get_walls_size(int number)
{
	return wall_size[number];
}

int pre_processing::get_walls_number()
{
	return wall_number;
}

Eigen::Matrix4f pre_processing::get_wall_transformation()
{
	return wall_transformation;
}

void pre_processing::set_walls_center(pcl::PointXYZ center,int number)
{
	wall_center[number] = center;
}

void pre_processing::set_walls_coefficients(pcl::PointXYZ coefficients,int number)
{
	wall_coefficients[number] = coefficients;
}

void pre_processing::set_walls_size(pcl::PointXYZ size,int number)
{
	wall_size[number] = size;
}

void pre_processing::set_walls_number(int number)
{
	wall_number=number;
}

Eigen::Matrix4f pre_processing::CreateRotateMatrix(Eigen::Vector3f before, Eigen::Vector3f after)
{
	before.normalize();
	after.normalize();

	float angle = acos(before.dot(after));
	//cout << "Angle: " << angle << endl;
	Eigen::Vector3f p_rotate = before.cross(after);
	p_rotate.normalize();

	Eigen::Matrix4f rotationMatrix = Eigen::Matrix4f::Identity();
	rotationMatrix(0, 0) = cos(angle) + p_rotate[0] * p_rotate[0] * (1 - cos(angle));
	rotationMatrix(0, 1) = p_rotate[0] * p_rotate[1] * (1 - cos(angle)) - p_rotate[2] * sin(angle);
	rotationMatrix(0, 2) = p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));


	rotationMatrix(1, 0) = p_rotate[2] * sin(angle) + p_rotate[0] * p_rotate[1] * (1 - cos(angle));
	rotationMatrix(1, 1) = cos(angle) + p_rotate[1] * p_rotate[1] * (1 - cos(angle));
	rotationMatrix(1, 2) = -p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));


	rotationMatrix(2, 0) = -p_rotate[1] * sin(angle) + p_rotate[0] * p_rotate[2] * (1 - cos(angle));
	rotationMatrix(2, 1) = p_rotate[0] * sin(angle) + p_rotate[1] * p_rotate[2] * (1 - cos(angle));
	rotationMatrix(2, 2) = cos(angle) + p_rotate[2] * p_rotate[2] * (1 - cos(angle));

	float roll, pitch, yaw;
	
	//cout << "Rotation Matrix: " << endl;
	//cout << rotationMatrix << endl;

	if (rotationMatrix(2, 1) < 1)
	{
		if (rotationMatrix(2, 1) > -1)
		{
			roll = asin(rotationMatrix(2, 1));
			yaw = atan2(-rotationMatrix(0, 1), rotationMatrix(1, 1));
			pitch = atan2(-rotationMatrix(2, 0), rotationMatrix(2, 2));

		}
		else
		{
			roll = -M_PI / 2;
			yaw = -atan2(rotationMatrix(0, 2), rotationMatrix(0, 0));
			pitch = 0;
		}
	}
	else
	{
		roll = M_PI / 2;
		yaw = atan2(rotationMatrix(0, 2), rotationMatrix(0, 0));
		pitch = 0;
	}

	//cout << "roll : " << roll / M_PI * 180 << endl;
	//cout << "pitch : " << pitch / M_PI * 180 << endl;
	//cout << "yaw : " << yaw / M_PI * 180 << endl;

	return rotationMatrix;
}

void pre_processing::filter_wrong_point(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_color, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_class)
{
	clock_t t;
	float offset = 0.025;

	vector<int> inliers(cloud_color->points.size(),0); //initialize index

	t = clock();

	//filter out the wrong points around ground plane
	float max_x, max_y, max_z;
	float min_x, min_y, min_z;

	max_x = plane_center.x + plane_size.x / 2 + offset;
	min_x = plane_center.x - plane_size.x / 2 - offset;

	max_y = plane_center.y + plane_size.y / 2 + offset;
	min_y = plane_center.y - plane_size.y / 2 - offset;

	max_z = plane_center.z + plane_size.z / 2 + offset;
	min_z = plane_center.z - plane_size.z / 2 - offset;

	for (int i = 0; i < cloud_color->points.size(); i++)
	{
		// if the point is in the box or not
		if (min_x <= cloud_color->points[i].x &&  cloud_color->points[i].x <= max_x && min_y <= cloud_color->points[i].y &&  cloud_color->points[i].y <= max_y && min_z <= cloud_color->points[i].z &&  cloud_color->points[i].z <= max_z)
		{
			inliers[i] = 1;
		}
	}
	
	//filter out the wrong points around wall plane
	for (int i = 0; i < wall_number; i++)
	{
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		Eigen::Vector3f normal;
		Eigen::Matrix4f temp_transformation = Eigen::Matrix4f::Identity();

		normal[0] = wall_coefficients[i].x;
		normal[1] = 0;
		normal[2] = wall_coefficients[i].z;

		temp_transformation(0, 3) = -wall_center[i].x;
		temp_transformation(1, 3) = -wall_center[i].y;
		temp_transformation(2, 3) = -wall_center[i].z;

		if (i != 0)
			temp_transformation = CreateRotateMatrix(normal,Eigen::Vector3f::UnitZ()) * temp_transformation;

		pcl::transformPointCloud(*cloud_color, *temp_cloud, temp_transformation);

		float max_x, max_y, max_z;
		float min_x, min_y, min_z;

		max_x = wall_size[i].x / 2 + offset;
		min_x = -wall_size[i].x / 2 - offset;

		max_y = wall_size[i].y / 2 + offset;
		min_y = -wall_size[i].y / 2 - offset;

		max_z = wall_size[i].z / 2 + offset;
		min_z = -wall_size[i].z / 2 - offset;

		for (int i = 0; i < temp_cloud->points.size(); i++)
		{
			// if the point is in the box or not
			if (min_x <= temp_cloud->points[i].x &&  temp_cloud->points[i].x <= max_x && min_y <= temp_cloud->points[i].y &&  temp_cloud->points[i].y <= max_y && min_z <= temp_cloud->points[i].z &&  temp_cloud->points[i].z <= max_z)
			{
				inliers[i] = 1;
			}
		}
	}
	
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr temp_class(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	for (int i = 0; i < inliers.size(); i++)
	{
		if (inliers[i] == 0)
		{
			temp->points.push_back(cloud_color->points[i]);
			temp_class->points.push_back(cloud_class->points[i]);
		}
	}

	*cloud_color = *temp;
	*cloud_class = *temp_class;

	cout << "Filter Wrong Point done!!" << " Time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;
}

int pre_processing::class_mapping(pcl::PointXYZRGBNormal point)
{
	float r, g, b;

	r = point.r;
	g = point.g;
	b = point.b;

	if (r == 0 && g == 128 && b == 128)
	{
		return 0;
	}
	else if (r == 250 && g == 50 && b == 50)
	{
		return 1;
	}
	else if (r == 102 && g == 0 && b == 204)
	{
		return 2;
	}
	else if (r == 50 && g == 50 && b == 250)
	{
		return 3;
	}
	else if (r == 220 && g == 220 && b == 220)
	{
		return 4;
	}
	else if (r == 255 && g == 69 && b == 20)
	{
		return 5;
	}
	else if (r == 255 && g == 20 && b == 127)
	{
		return 6;
	}
	else if (r == 50 && g == 50 && b == 150)
	{
		return 7;
	}
	else if (r == 222 && g == 180 && b == 140)
	{
		return 8;
	}
	else if (r == 50 && g == 250 && b == 50)
	{
		return 9;
	}
	else if (r == 255 && g == 215 && b == 0)
	{
		return 10;
	}
	else if (r == 150 && g == 150 && b == 150)
	{
		return 11;
	}
	else if (r == 0 && g == 255 && b == 255)
	{
		return 12;
	}
	else
	{
		return 13;
	}
}

pcl::PointCloud<pcl::PointXYZRGBNormal> pre_processing::refine_probability(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_color , pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_class)
{
	clock_t t;
	pcl::PointCloud<pcl::PointXYZRGBNormal> new_cloud;

	pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
	vector<int> pointIdxRadiusSearch;
	vector<float> pointRadiusSquaredDistance;

	t = clock();

	kdtree.setInputCloud(cloud_color);
	pcl::copyPointCloud(*cloud_class, new_cloud);

	float radius = 0.10;
	float color_normalize = 255 * 255 * 3;

	for (int n = 0; n < cloud_color->points.size(); n++)
	{
		float new_P[13] = { 0.0 };
		//target point color
		float p1_r, p1_g, p1_b; 
		p1_r = cloud_color->points[n].r;
		p1_g = cloud_color->points[n].g;
		p1_b = cloud_color->points[n].b;

		if (kdtree.radiusSearch(cloud_color->points[n], radius, pointIdxRadiusSearch, pointRadiusSquaredDistance))  // Euclidean Distance < radius is neighborhood. (Radius)
		{
			for (int i = 0; i < pointIdxRadiusSearch.size(); i++)
			{
				int category = 0;
				float color_distance=0.0;
				float Euclidean_distance = 0.0;

				//neiboring point color
				float p2_r, p2_g, p2_b;
				p2_r = cloud_color->points[pointIdxRadiusSearch[i]].r;
				p2_g = cloud_color->points[pointIdxRadiusSearch[i]].g;
				p2_b = cloud_color->points[pointIdxRadiusSearch[i]].b;

				category = class_mapping(cloud_class->points[pointIdxRadiusSearch[i]]);

				if (category < 13)
				{
					Euclidean_distance = 1.0 - sqrt(pointRadiusSquaredDistance[i]) / radius;  //The weight of the Euclidean distance

					color_distance = 1.0 - sqrt((p1_r - p2_r)*(p1_r - p2_r) + (p1_g - p2_g)*(p1_g - p2_g) + (p1_b - p2_b)*(p1_b - p2_b)) / color_normalize; //the weight of the color distance

					if (Euclidean_distance < 0 || color_distance < 0)
					{
						cout << Euclidean_distance << " " << color_distance << endl;
					}

					new_P[category] += Euclidean_distance * color_distance;
				}
			}
		}
		int target_class = class_mapping(cloud_class->points[n]);

		if(target_class <13)
			new_P[target_class] += 1.0; //the weight of target point

		float max = 0.0;
		int index = 5;
		for (int i = 0; i < 13; i++)
		{
			if (new_P[i] > max)
			{
				max = new_P[i];
				index = i;
			}
		}

		if (index == 0)
		{
			new_cloud.points[n].r = 0;
			new_cloud.points[n].g = 128;
			new_cloud.points[n].b = 128;
		}
		else if (index == 1)
		{
			new_cloud.points[n].r = 250;
			new_cloud.points[n].g = 50;
			new_cloud.points[n].b = 50;
		}
		else if (index == 2)
		{
			new_cloud.points[n].r = 102;
			new_cloud.points[n].g = 0;
			new_cloud.points[n].b = 204;
		}
		else if (index == 3)
		{
			new_cloud.points[n].r = 50;
			new_cloud.points[n].g = 50;
			new_cloud.points[n].b = 250;
		}
		else if (index == 4)
		{
			new_cloud.points[n].r = 220;
			new_cloud.points[n].g = 220;
			new_cloud.points[n].b = 220;
		}
		else if (index == 5)
		{
			new_cloud.points[n].r = 255;
			new_cloud.points[n].g = 69;
			new_cloud.points[n].b = 20;
		}
		else if (index == 6)
		{
			//new_cloud.points[n].r = 255;
			//new_cloud.points[n].g = 69;
			//new_cloud.points[n].b = 20;
			new_cloud.points[n].r = 255;
			new_cloud.points[n].g = 20;
			new_cloud.points[n].b = 127;
		}
		else if (index == 7)
		{
			new_cloud.points[n].r = 50;
			new_cloud.points[n].g = 50;
			new_cloud.points[n].b = 150;
		}
		else if (index == 8)
		{
			new_cloud.points[n].r = 222;
			new_cloud.points[n].g = 180;
			new_cloud.points[n].b = 140;
		}
		else if (index == 9)
		{
			new_cloud.points[n].r = 50;
			new_cloud.points[n].g = 250;
			new_cloud.points[n].b = 50;
		}
		else if (index == 10)
		{
			new_cloud.points[n].r = 255;
			new_cloud.points[n].g = 215;
			new_cloud.points[n].b = 0;
		}
		else if (index == 11)
		{
			new_cloud.points[n].r = 150;
			new_cloud.points[n].g = 150;
			new_cloud.points[n].b = 150;
		}
		else if (index == 12)
		{
			new_cloud.points[n].r = 0;
			new_cloud.points[n].g = 255;
			new_cloud.points[n].b = 255;
		}
	}

	cout << "Refine Probability done!!" << " Time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;

	return new_cloud;
}

void pre_processing::remove_invalid_point(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_color, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_class)
{
	pcl::PointCloud<pcl::PointXYZRGBNormal> temp_class;
	pcl::PointCloud<pcl::PointXYZRGBNormal> temp_color;

	pcl::PointCloud<pcl::PointXYZRGBNormal> category[13];
	pcl::PointCloud<pcl::PointXYZRGBNormal> color[13];

	clock_t t;

	t = clock();

	for (int n = 0; n < cloud_class->points.size(); n++)
	{
		float r, g, b;

		r = cloud_class->points[n].r;
		g = cloud_class->points[n].g;
		b = cloud_class->points[n].b;

		if (r == 0 && g == 128 && b == 128)
		{
			category[0].push_back(cloud_class->points[n]);
			color[0].push_back(cloud_color->points[n]);
		}
		else if (r == 250 && g == 50 && b == 50)
		{
			category[1].push_back(cloud_class->points[n]);
			color[1].push_back(cloud_color->points[n]);
		}
		else if (r == 102 && g == 0 && b == 204)
		{
			category[2].push_back(cloud_class->points[n]);
			color[2].push_back(cloud_color->points[n]);
		}
		else if (r == 50 && g == 50 && b == 250)
		{
			category[3].push_back(cloud_class->points[n]);
			color[3].push_back(cloud_color->points[n]);
		}
		else if (r == 220 && g == 220 && b == 220)
		{
			category[4].push_back(cloud_class->points[n]);
			color[4].push_back(cloud_color->points[n]);
		}
		else if (r == 255 && g == 69 && b == 20)
		{
			category[5].push_back(cloud_class->points[n]);
			color[5].push_back(cloud_color->points[n]);
		}
		else if (r == 255 && g == 20 && b == 127)
		{
			category[6].push_back(cloud_class->points[n]);
			color[6].push_back(cloud_color->points[n]);
		}
		else if (r == 50 && g == 50 && b == 150)
		{
			category[7].push_back(cloud_class->points[n]);
			color[7].push_back(cloud_color->points[n]);
		}
		else if (r == 222 && g == 180 && b == 140)
		{
			category[8].push_back(cloud_class->points[n]);
			color[8].push_back(cloud_color->points[n]);
		}
		else if (r == 50 && g == 250 && b == 50)
		{
			category[9].push_back(cloud_class->points[n]);
			color[9].push_back(cloud_color->points[n]);
		}
		else if (r == 255 && g == 215 && b == 0)
		{
			category[10].push_back(cloud_class->points[n]);
			color[10].push_back(cloud_color->points[n]);
		}
		else if (r == 150 && g == 150 && b == 150)
		{
			category[11].push_back(cloud_class->points[n]);
			color[11].push_back(cloud_color->points[n]);
		}
		else if (r == 0 && g == 255 && b == 255)
		{
			category[12].push_back(cloud_class->points[n]);
			color[12].push_back(cloud_color->points[n]);
		}
	}
	cout << "remove_invalid_point done!!" << " Time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;

	t = clock();

	pcl::console::print_highlight("Downsampling...\n");
	pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid;
	float leaf = 0.03f;
	grid.setLeafSize(leaf, leaf, leaf);

	for (int i = 0; i < 13; i++)
	{
		grid.setInputCloud(category[i].makeShared());
		grid.filter(category[i]);

		grid.setInputCloud(color[i].makeShared());
		grid.filter(color[i]);

		temp_color += color[i];
		temp_class += category[i];

		//cout << "C" <<i<<": "<< category[i].points.size() << endl;
	}

	*cloud_color = temp_color;
	*cloud_class = temp_class;

	cout << "After Downsampling and removing invalid point:" << cloud_color->points.size() << endl;

	cout << "Downsampling done!!" << " Time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;
}

void pre_processing::check(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
	int count = 0;
	for (int i = 0; i < cloud->points.size(); i++)
	{
		float r, g, b;

		r = cloud->points[i].r;
		g = cloud->points[i].g;
		b = cloud->points[i].b;

		if (r == 0 && g == 128 && b == 128)
		{
			continue;
		}
		else if (r == 250 && g == 50 && b == 50)
		{
			continue;
		}
		else if (r == 102 && g == 0 && b == 204)
		{
			continue;
		}
		else if (r == 50 && g == 50 && b == 250)
		{
			continue;
		}
		else if (r == 220 && g == 220 && b == 220)
		{
			continue;
		}
		else if (r == 255 && g == 69 && b == 20)
		{
			continue;
		}
		else if (r == 255 && g == 20 && b == 127)
		{
			continue;
		}
		else if (r == 50 && g == 50 && b == 150)
		{
			continue;
		}
		else if (r == 222 && g == 180 && b == 140)
		{
			continue;
		}
		else if (r == 50 && g == 250 && b == 50)
		{
			continue;
		}
		else if (r == 255 && g == 215 && b == 0)
		{
			continue;
		}
		else if (r == 150 && g == 150 && b == 150)
		{
			continue;
		}
		else if (r == 0 && g == 255 && b == 255)
		{
			continue;
		}
		else
		{
			count++;
		}
	}
	cout << "NaN: " << count << endl;
}