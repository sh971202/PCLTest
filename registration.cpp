#include <iostream>
#include <string>
#include <vector>
#include <time.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include "registration.h"


using namespace std;

void registration::Euclidean_Cluster(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud)
{
	clock_t t;
	pcl::PointCloud<pcl::PointXYZ>::Ptr copy_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	t = clock();
	/*
	pcl::console::print_highlight("Downsampling...\n");
	pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid;
	float leaf = 0.03f;
	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(cloud);
	grid.filter(*cloud);
	cout << "Scene points:" << cloud->size() << endl;
	*/

	pcl::copyPointCloud(*cloud, *copy_cloud); //copy original cloud

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	tree->setInputCloud(copy_cloud);

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;

	ec.setClusterTolerance(0.08); // 3.5cm
	ec.setMinClusterSize(300);
	ec.setMaxClusterSize(100000);
	ec.setSearchMethod(tree);
	ec.setInputCloud(copy_cloud);
	ec.extract(cluster_indices);
	
	total_object_number = 0; // number of object
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
	{
		pcl::PointCloud<pcl::PointXYZRGBNormal> cloud_cluster;
		for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
		{
			cloud_cluster.points.push_back(cloud->points[*pit]);  // copy extract cloud
		}
		cloud_cluster.width = cloud_cluster.points.size();
		cloud_cluster.height = 1;
		cloud_cluster.is_dense = true;

		cout << "PointCloud representing the Cluster: " << cloud_cluster.points.size() << " data points." << endl;
		/*
		pcl::console::print_highlight("Downsampling...\n");
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		*temp = cloud_cluster;
		pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid;
		//float leaf;
		leaf = 0.03f;
		grid.setLeafSize(leaf, leaf, leaf);
		grid.setInputCloud(temp);
		grid.filter(cloud_cluster);
		cout << "Scene points:" << cloud_cluster.size() << endl;*/

		object_cloud.push_back(cloud_cluster); // put object into object array

		total_object_number++;
	}

	//execution time
	cout << "Euclidean_Cluster time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;
}

pcl::PointCloud<pcl::PointXYZRGBNormal> registration::get_object_cloud(int number)
{
	return object_cloud[number];
}

int registration::get_object_number()
{
	return total_object_number;
}

void registration::find_object_plane(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, string id, vector<Plane>& planes,float distrance_threshold,float size_threshold)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>), cloud_f(new pcl::PointCloud<pcl::PointXYZ>), cloud_p(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud, *cloud_filtered); //copy original cloud

	pcl::PointCloud<pcl::Normal>::Ptr cloud_normal(new pcl::PointCloud<pcl::Normal>), normal_f(new pcl::PointCloud<pcl::Normal>);
	pcl::copyPointCloud(*cloud, *cloud_normal); //copy original cloud normal

	clock_t t;

	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::ModelCoefficients plane_coefficients;

	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;

	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
	seg.setNormalDistanceWeight(0.03);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(distrance_threshold);
	seg.setMaxIterations(100);
	//seg.setProbability(0.8);
	//seg.set

	/*
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(0.01);
	*/
	// Create the filtering object
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	pcl::ExtractIndices<pcl::Normal> extract_normals;

	int nr_points = cloud->points.size();
	int number = 0, op = 0;

	while (cloud_filtered->points.size() > size_threshold * nr_points)
	{
		t = clock();
		// Segment the largest planar component from the remaining cloud
		seg.setInputCloud(cloud_filtered);
		seg.setInputNormals(cloud_normal);
		seg.segment(*inliers, plane_coefficients);

		if (inliers->indices.size() == 0)
		{
			cout << "Could not estimate a planar model for the given dataset." << endl;
			break;
		}

		// Extract the inliers
		extract.setInputCloud(cloud_filtered);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_p);

		number++;
		if (cloud_p->width * cloud_p->height < nr_points * size_threshold)
		{
			cout << "The Point size is too small." << endl;
			cout << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << endl << endl;
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tc_handler(cloud_p, 150, 150, 150);
			viewer.addPointCloud(cloud_p, tc_handler,"rest_cloud" + to_string(number));
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "rest_cloud" + to_string(number));
			//extract the rest of points
			extract.setNegative(true);
			extract.filter(*cloud_f);
			cloud_filtered.swap(cloud_f);
			//extract the rest of normals
			extract_normals.setInputCloud(cloud_normal);
			extract_normals.setIndices(inliers);
			extract_normals.setNegative(true);
			extract_normals.filter(*normal_f);
			cloud_normal.swap(normal_f);
			//op = 1;
			break;
		}
		

		//execution time
		cout << "Plane segmentation time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;
		cout << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << endl;
		cout << "Model Coefficients: " << plane_coefficients.values[0] << " " << plane_coefficients.values[1] << " " << plane_coefficients.values[2] << " " << plane_coefficients.values[3] << " " << endl;


		//normal vector of the plane
		Eigen::Vector3f N;
		pcl::PointXYZ box_center;

		//plane normal
		N[0] = plane_coefficients.values[0];
		N[1] = plane_coefficients.values[1];
		N[2] = plane_coefficients.values[2];

		//Eigen::Matrix4f T = process.CreateRotateMatrix(N, Eigen::Vector3f::UnitZ());

		//pcl::transformPointCloud(*cloud_p, *cloud_p, T);

		pcl::PointXYZ minpt, maxpt;
		pcl::getMinMax3D(*cloud_p, minpt, maxpt);

		Eigen::Vector4f new_min, new_max;

		new_min[0] = minpt.x;
		new_min[1] = minpt.y;
		new_min[2] = minpt.z;
		new_min[3] = 1;

		new_max[0] = maxpt.x;
		new_max[1] = maxpt.y;
		new_max[2] = maxpt.z;
		new_max[3] = 1;

		//new_min = T.inverse() * new_min;
		//new_max = T.inverse() * new_max;

		minpt.x = new_min[0];
		minpt.y = new_min[1];
		minpt.z = new_min[2];

		maxpt.x = new_max[0];
		maxpt.y = new_max[1];
		maxpt.z = new_max[2];


		pcl::PointXYZ pt1(minpt.x, minpt.y, minpt.z);
		pcl::PointXYZ pt2(minpt.x, minpt.y, maxpt.z);
		pcl::PointXYZ pt3(maxpt.x, minpt.y, maxpt.z);
		pcl::PointXYZ pt4(maxpt.x, minpt.y, minpt.z);
		pcl::PointXYZ pt5(minpt.x, maxpt.y, minpt.z);
		pcl::PointXYZ pt6(minpt.x, maxpt.y, maxpt.z);
		pcl::PointXYZ pt7(maxpt.x, maxpt.y, maxpt.z);
		pcl::PointXYZ pt8(maxpt.x, maxpt.y, minpt.z);

		/*
		viewer.addLine(pt1, pt2, 1.0, 0.0, 0.0, "1 edge" + id + "N" + to_string(number));
		viewer.addLine(pt1, pt4, 1.0, 0.0, 0.0, "2 edge" + id + "N" + to_string(number));
		viewer.addLine(pt1, pt5, 1.0, 0.0, 0.0, "3 edge" + id + "N" + to_string(number));
		viewer.addLine(pt5, pt6, 1.0, 0.0, 0.0, "4 edge" + id + "N" + to_string(number));
		viewer.addLine(pt5, pt8, 1.0, 0.0, 0.0, "5 edge" + id + "N" + to_string(number));
		viewer.addLine(pt2, pt6, 1.0, 0.0, 0.0, "6 edge" + id + "N" + to_string(number));
		viewer.addLine(pt6, pt7, 1.0, 0.0, 0.0, "7 edge" + id + "N" + to_string(number));
		viewer.addLine(pt7, pt8, 1.0, 0.0, 0.0, "8 edge" + id + "N" + to_string(number));
		viewer.addLine(pt2, pt3, 1.0, 0.0, 0.0, "9 edge" + id + "N" + to_string(number));
		viewer.addLine(pt4, pt8, 1.0, 0.0, 0.0, "10 edge" + id + "N" + to_string(number));
		viewer.addLine(pt3, pt4, 1.0, 0.0, 0.0, "11 edge" + id + "N" + to_string(number));
		viewer.addLine(pt3, pt7, 1.0, 0.0, 0.0, "12 edge" + id + "N" + to_string(number));
		*/
		box_center.x = (pt1.x + pt2.x + pt3.x + pt4.x + pt5.x + pt6.x + pt7.x + pt8.x) / 8;  // x position of bounding box 
		box_center.y = (pt1.y + pt2.y + pt3.y + pt4.y + pt5.y + pt6.y + pt7.y + pt8.y) / 8;  // y position of bounding box 
		box_center.z = (pt1.z + pt2.z + pt3.z + pt4.z + pt5.z + pt6.z + pt7.z + pt8.z) / 8;  // z position of bounding box 

		cout << "Max-Min(x,y,z): " << maxpt.x - minpt.x << "," << maxpt.y - minpt.y << "," << maxpt.z - minpt.z << endl;
		cout << "box_center(x,y,z): " << box_center.x << "," << box_center.y << "," << box_center.z << endl;

		//pcl::PointXYZ end(box_center.x + plane_coefficients.values[0], box_center.y + plane_coefficients.values[1], box_center.z + plane_coefficients.values[2]);
		//viewer.addArrow(end, box_center, 0.0, 0.0, 1.0, false, "arrow_z1" + id +to_string(number));

		Plane temp_plane;

		temp_plane.normal = N;
		temp_plane.box_center = box_center;
		temp_plane.height = maxpt.y - minpt.y;
		temp_plane.width = maxpt.x - minpt.x;
		planes.push_back(temp_plane);

		//pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ>tc_handler(cloud_p);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tc_handler(cloud_p, 255 / (number+1), 0, 100 * number);
		viewer.addPointCloud(cloud_p, tc_handler,"cloud" + id + to_string(number));
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud" + id + to_string(number));

		//extract the rest of points
		extract.setNegative(true);
		extract.filter(*cloud_f);
		cloud_filtered.swap(cloud_f);
		//extract the rest of normals
		extract_normals.setInputCloud(cloud_normal);
		extract_normals.setIndices(inliers);
		extract_normals.setNegative(true);
		extract_normals.filter(*normal_f);
		cloud_normal.swap(normal_f);

		cout << endl;
	}
	if (op == 0)
	{
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tc_handler(cloud_filtered, 150, 150, 150);
		viewer.addPointCloud(cloud_filtered, tc_handler,"rest_cloud");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "rest_cloud");
	}
}

double registration::get_model_transformation(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud)
{
	clock_t t;
	Eigen::Matrix4f final_transformation_matrix = Eigen::Matrix4f::Identity();
	pcl::PointCloud<pcl::PointXYZ>::Ptr copy_target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr copy_model_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*target_cloud, *copy_target_cloud); //copy original target cloud
	pcl::copyPointCloud(*model_cloud, *copy_model_cloud); //copy original model cloud
	cout << "ICP start!!" << endl;

	/*
	//align two point clouds mass
	Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();
	Eigen::Vector4f centroid;
	pcl::compute3DCentroid(*copy_target_cloud, centroid);
	translate(0, 3) = centroid[0];
	translate(1, 3) = centroid[1];
	translate(2, 3) = centroid[2];
	pcl::compute3DCentroid(*copy_model_cloud, centroid);
	translate(0, 3) -= centroid[0];
	translate(1, 3) -= centroid[1];
	translate(2, 3) -= centroid[2];
	
	pcl::transformPointCloud(*copy_model_cloud, *copy_model_cloud, translate);
	cout << "translate:" << endl;
	cout << translate << endl;
	final_transformation_matrix = translate * final_transformation_matrix;
	*/
	t = clock();
	// The Iterative Closest Point algorithm
	Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(copy_target_cloud);
	icp.setInputTarget(copy_model_cloud);
	
	//icp.setInputSource(copy_model_cloud);
	//icp.setInputTarget(copy_target_cloud);
	icp.setMaximumIterations(1);
	icp.setMaxCorrespondenceDistance(0.05);
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.0001);

	pcl::PointCloud<pcl::PointXYZ>::Ptr final_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	icp.align(*copy_target_cloud);
	final_transformation_matrix *= icp.getFinalTransformation().cast<float>();
	/*
	for (int i = 0; i < 5; i++)
	{
		icp.align(*copy_target_cloud);
		transformation_matrix *= icp.getFinalTransformation().cast<float>();
		final_transformation_matrix = transformation_matrix.inverse() * final_transformation_matrix;
		pcl::transformPointCloud(*copy_model_cloud, *copy_model_cloud, transformation_matrix.inverse());
	}
	
	final_transformation_matrix(0, 1) = 0;
	final_transformation_matrix(1, 0) = 0;
	final_transformation_matrix(1, 1) = 1;
	final_transformation_matrix(1, 2) = 0;
	final_transformation_matrix(2, 1) = 0;
	final_transformation_matrix(1, 3) = 0;
	*/

	cout << "ICP time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;
	cout << "Score: " << icp.getFitnessScore() << endl;
	cout << "Has converged: " << icp.hasConverged() << endl;
	cout << "Tanformation:" << endl;
	cout << final_transformation_matrix << endl;

	return icp.getFitnessScore();
}