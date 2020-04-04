#include <iostream>
#include <fstream>
#include <string>
#include <time.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/obj_io.h>
#include <pcl/PolygonMesh.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/common/transformation_from_correspondences.h>
#include <pcl/common/centroid.h>

#include <pcl/filters/project_inliers.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>

#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/esf.h>
#include<pcl/features/principal_curvatures.h>
#include <pcl/features/fpfh_omp.h>

#include "pre_processing.h"
#include "registration.h"

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkRenderingOpenGL);

using namespace std;

pre_processing process;
registration reg;

bool next_iteration = false;


void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event, void* nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		next_iteration = true;
}

void viewerOneOff(pcl::visualization::PCLVisualizer& viewer)
{
	viewer.setBackgroundColor(255, 255, 255);
}

pcl::PointCloud<pcl::PointXYZ>::Ptr Harris3D_key_point(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, vector<Plane> object_planes, string id)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempc_loud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr normal(new pcl::PointCloud<pcl::Normal>);

	pcl::copyPointCloud(*cloud, *tempc_loud); //copy original cloud
	pcl::copyPointCloud(*cloud, *normal); //copy original cloud normal

	pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZI>);

	pcl::HarrisKeypoint3D<pcl::PointXYZ, pcl::PointXYZI, pcl::Normal> harris;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
	pcl::PointIndicesConstPtr key_point;
	clock_t t;
	t = clock();
	cout << "Key Point extract start!!" << endl;
	harris.setInputCloud(tempc_loud);
	harris.setNormals(normal);
	//harris.setSearchMethod(tree);
	harris.setNonMaxSupression(true);
	harris.setRadius(0.08f);
	//harris.setRadiusSearch(0.0f);
	//harris.setKSearch(50);
	//harris.setRefine(false);
	//harris.setNumberOfThreads(4);
	harris.setThreshold(0.002f);
	harris.compute(*cloud_out);
	key_point = harris.getKeypointsIndices();

	cout << "ksize: " << key_point->indices.size() << endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_harris(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointXYZ point;
	for (int i = 0; i < cloud_out->size(); i++)
	{
		point.x = cloud_out->at(i).x;
		point.y = cloud_out->at(i).y;
		point.z = cloud_out->at(i).z;
		//cout << "I: " << cloud_out->points[i].intensity << endl;

		int op = 0;
		for (int j = 0; j < cloud_harris->size(); j++)
		{
			double dis = sqrt(pow(point.x - cloud_harris->points[j].x, 2) + pow(point.y - cloud_harris->points[j].y, 2) + pow(point.z - cloud_harris->points[j].z, 2));
			if (dis < 0.05)
			{
				op = 1;
				break;
			}
		}
		if (op == 0)
		{
			cloud_harris->push_back(point);
		}

	}

	cout << cloud_harris->size() << endl;

	//---------PCA filter--------

	int count = 0;

	for (pcl::PointCloud<pcl::PointXYZ>::iterator K = cloud_harris->begin(); K != cloud_harris->end();)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
		vector<int> pointIdxRadiusSearch;
		vector<float> pointRadiusSquaredDistance;

		kdtree.setInputCloud(tempc_loud);

		if (kdtree.radiusSearch(*K, 0.15f, pointIdxRadiusSearch, pointRadiusSquaredDistance))  // Euclidean Distance < 0.15m is neighborhood. (Radius)
		{
			if (pointIdxRadiusSearch.size() < 10)
			{
				K = cloud_harris->erase(K);
				continue;
			}

			for (int j = 0; j < pointIdxRadiusSearch.size(); j++)
			{
				temp->push_back(tempc_loud->points[pointIdxRadiusSearch[j]]);
			}
			Eigen::Vector4f pcaCentroid;

			pcl::compute3DCentroid(*temp, pcaCentroid);
			Eigen::Matrix3f covariance;
			pcl::computeCovarianceMatrixNormalized(*temp, pcaCentroid, covariance);
			//pcl::computeCovarianceMatrixNormalized(*normal, Eigen::Vector4f(0, 0, 0, 0), covariance);
			Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
			Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
			Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
			//Eigen::Matrix3f eigenVectorsPCA = pca_copy.getEigenVectors();
			//Eigen::Vector3f eigenValuesPCA = pca_copy.getEigenValues();

			cout << "PCA: " << eigenValuesPCA[0] << " " << eigenValuesPCA[1] << " " << eigenValuesPCA[2] << endl;

			float L1 = eigenValuesPCA[0];
			float L2 = eigenValuesPCA[1];
			float L3 = eigenValuesPCA[2];

			float RATIO = (L1 * L2) / (L3*(L1 + L2) + L2 * L2);

			cout << "RATIO: " << RATIO << endl;

			pcl::PointCloud<pcl::PointXYZ>::Ptr temp_pca(new pcl::PointCloud<pcl::PointXYZ>);

			temp_pca->points.push_back(*K);

			if (RATIO > 0.04)
			{
				K++;

				//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> harris_color_handler_s(temp_pca, 0, 255, 0);
				//viewer.addPointCloud(temp_pca, harris_color_handler_s, "harris_scene" + to_string(count) + "ID" + id);
				//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "harris_scene" + to_string(count) + "ID" + id);
			}
			else
			{
				K = cloud_harris->erase(K);

				//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> harris_color_handler_s(temp_pca, 255, 0, 0);
				//viewer.addPointCloud(temp_pca, harris_color_handler_s, "harris_scene" + to_string(count) + "ID" + id);
				//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "harris_scene" + to_string(count) + "ID" + id);
			}
			count++;
		}
		else
		{
			K = cloud_harris->erase(K);
			continue;
		}
	}

	//pcl::PointCloud<pcl::PointXYZ>::Ptr temp_pca(new pcl::PointCloud<pcl::PointXYZ>);

	//temp_pca->points.push_back(cloud_harris->points[0]);

	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> harris_color_handler_s(temp_pca, 0, 255, 0);
	//viewer.addPointCloud(temp_pca, harris_color_handler_s, "harris_scene" + to_string(count) + "ID" + id);
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "harris_scene" + to_string(count) + "ID" + id);
	//---------PCA filter--------


	/*
	for (int i = 0; i < cloud_harris->size(); i++)
	{
	cout << cloud_harris->points[i].y << endl;
	}
	*/
	cout << " Time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;

	/*
	for (int i = 0; i < cloud_harris->size(); i++)
	{
	viewer.addSphere(cloud_harris->points[i], 0.15, "S" + to_string(i)+id);
	}
	*/

	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> harris_color_handler(cloud_harris, 255, 0, 0);
	//viewer.addPointCloud(cloud_harris, harris_color_handler, "harris" + id);
	//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "harris" + id);

	cout << "After PCAfilter: " << cloud_harris->size() << endl;

	return cloud_harris;
}

vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> Kdescriptor(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr Keypoint, string id)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempc_loud(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::copyPointCloud(*cloud, *tempc_loud); //copy original cloud

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	vector<int> pointIdxRadiusSearch;
	vector<float> pointRadiusSquaredDistance;

	vector<int> pointIdxNKNSearch(50);
	vector<float> pointNKNSquaredDistance(50);


	vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> descriptors;

	kdtree.setInputCloud(tempc_loud);

	for (int i = 0; i < Keypoint->size(); i++)
	{
		/*
		if (kdtree.nearestKSearch(Keypoint->points[i],50, pointIdxNKNSearch, pointNKNSquaredDistance))  // Top 50 minimum Euclidean Distance (Knearst)
		{
		//cout << pointIdxRadiusSearch.size() << endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);

		for (int j = 0; j < pointIdxNKNSearch.size(); j++)
		{
		temp->push_back(cloud->points[pointIdxNKNSearch[j]]);
		}

		descriptors.push_back(temp);

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> temp_color_handler(temp, 0, 255, 0);
		viewer.addPointCloud(temp, temp_color_handler, "kdescriptor" + to_string(i) + id);
		}
		*/

		if (kdtree.radiusSearch(Keypoint->points[i], 0.15f, pointIdxRadiusSearch, pointRadiusSquaredDistance))  // Euclidean Distance < 0.15m is neighborhood. (Radius)
		{
			//cout << pointIdxRadiusSearch.size() << endl;
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

			for (int j = 0; j < pointIdxRadiusSearch.size(); j++)
			{
				temp->push_back(cloud->points[pointIdxRadiusSearch[j]]);
			}

			descriptors.push_back(temp);

			//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> temp_color_handler(temp, 0, 255, 0);
			//viewer.addPointCloud(temp, temp_color_handler, "kdescriptor" + to_string(i) + id);
			//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "kdescriptor" + to_string(i) + id);
			//viewer.addPointCloudNormals<pcl::PointXYZRGBNormal>(temp, 1, 0.02, "normal" + to_string(i) + id);
		}

	}
	//cout << "i:" <<descriptors.size()<< endl;
	return descriptors;
}

// Align a rigid object to a scene with clutter and occlusions
int main(int argc, char **argv)
{
	// Point clouds
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr object(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr scene(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	
	pcl::io::loadPLYFile<pcl::PointXYZRGBNormal>("./pc1.ply", *object);
	pcl::io::loadPLYFile<pcl::PointXYZRGBNormal>("./cloud-1.ply", *scene);
	

	// Downsample
	pcl::console::print_highlight("Downsampling...\n");
	pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid;
	const float leaf = 0.005f;
	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(object);
	grid.filter(*object);
	grid.setInputCloud(scene);
	grid.filter(*scene);

	int count = 0;
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL); //add keyboard event
																		  //viewer.addCoordinateSystem(0.3, "coordinate", 0);
	viewer.initCameraParameters();
	//This will only get called once
	viewerOneOff(viewer);

	vector<Plane> object_plane;
	reg.find_object_plane(viewer, object, to_string(count), object_plane, 0.02, 0.1);

	for (int i = 0; i < object_plane.size(); i++)
	{
		float angle = acos(Eigen::Vector3f::UnitY().dot(object_plane[i].normal));

		if (angle * 180 / M_PI > 90.0)
			object_plane[i].normal = object_plane[i].normal * -1;
	}

	
	pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints(new pcl::PointCloud<pcl::PointXYZ>);
	vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> scene_descriptors;
	scene_keypoints = Harris3D_key_point(viewer, scene, object_plane, to_string(count));
	scene_descriptors = Kdescriptor(viewer, scene, scene_keypoints, to_string(count));
	PCA_visualize(viewer, scene_descriptors, scene_keypoints, to_string(count));

	system("pause");
	return (0);
}