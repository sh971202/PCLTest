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

#include <pcl/keypoints/harris_3d.h>
#include <pcl/features/esf.h>

#include "pre_processing.h"
#include "registration.h"
#include "save_model.h"
#include "./json/save_json.h"


#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkRenderingOpenGL);

using namespace std;

pcl::PointCloud<pcl::PointXYZ>::Ptr Harris3D_key_point(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, string id)
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
	harris.setRadius(0.12f);
	//harris.setRadiusSearch(0.0f);
	//harris.setKSearch(50);
	//harris.setRefine(false);
	//harris.setNumberOfThreads(4);
	harris.setThreshold(0.9f);
	harris.compute(*cloud_out);
	key_point = harris.getKeypointsIndices();

	cout << "ksize: " << key_point->indices.size() << endl;

	pcl::PointCloud<pcl::PointXYZI>::Ptr temp_keypints(new pcl::PointCloud<pcl::PointXYZI>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_harris(new pcl::PointCloud<pcl::PointXYZ>);

	pcl::PointXYZI point;
	for (int i = 0; i < cloud_out->size(); i++)
	{
		point.x = cloud_out->at(i).x;
		point.y = cloud_out->at(i).y;
		point.z = cloud_out->at(i).z;
		point.intensity = cloud_out->at(i).intensity;
		cout << "I: " << cloud_out->points[i].intensity << endl;

		int op = 0;
		for (vector<pcl::PointXYZI>::iterator j= temp_keypints->begin(); j != temp_keypints->end();)
		{
			double dis = sqrt(pow(point.x - j->x, 2) + pow(point.y - j->y, 2) + pow(point.z - j->z, 2));
			if (dis < 0.1)
			{
				if (point.intensity > j->intensity)
				{
					j = temp_keypints->erase(j);
				}
				else
				{
					op = 1;
					break;
				}

			}
			else
			{
				j++;
			}
		}
		if (op == 0)
		{
			temp_keypints->push_back(point);
		}

	}

	pcl::copyPointCloud(*temp_keypints, *cloud_harris); //copy original cloud

	for (int i = 0; i < cloud_harris->size(); i++)
	{
		cout << i << ": " << cloud_harris->points[i].y << endl;
	}

	//---------PCA filter--------
	
	int count = 0;
	/*
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

			//if(count == 4)
			{
				//K->y -= 0.02;
				//K++;

				//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> harris_color_handler_s(temp_pca, 0, 255, 0);
				//viewer.addPointCloud(temp_pca, harris_color_handler_s, "harris_scene" + to_string(count) + "ID" + id);
				//viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "harris_scene" + to_string(count) + "ID" + id);
			}
			if (count == 1)
			{
				K = cloud_harris->erase(K);
			}
			else if (count == 6)
			{
				K = cloud_harris->erase(K);
			}
			else if (count == 12)
			{
				K = cloud_harris->erase(K);
			}
			else if (RATIO > 0.08)
			{
				K++;
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
	*/
	//---------PCA filter--------
	

	cout << cloud_harris->size() << endl;
	
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
		if (kdtree.radiusSearch(Keypoint->points[i], 0.15f, pointIdxRadiusSearch, pointRadiusSquaredDistance))  // Euclidean Distance < 0.1m is neighborhood. (Radius)
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
			//viewer.addPointCloudNormals<pcl::PointXYZRGBNormal>(temp, 1, 0.02, "normal" + to_string(i) + id);
		}
		
	}
	//cout << "i:" <<descriptors.size()<< endl;
	return descriptors;
}

pcl::ESFSignature640 ESFdescriptor(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esf;
	pcl::PointCloud<pcl::ESFSignature640>::Ptr model1_descriptor(new pcl::PointCloud<pcl::ESFSignature640>);
	clock_t t;

	t = clock();

	// ESF estimation object.
	esf.setInputCloud(cloud);
	esf.compute(*model1_descriptor);

	vector<float> Min_D;
	vector<int> index;

	for (int k = 0; k < 4; k++)
	{

		// Object for storing the ESF descriptor.
		pcl::PointCloud<pcl::ESFSignature640>::Ptr model2_descriptor(new pcl::PointCloud<pcl::ESFSignature640>);

		// ESF estimation object. 
		esf.setInputCloud(cloud);

		esf.compute(*model2_descriptor);

		float dis = 0.0;
		for (int i = 0; i < model1_descriptor->points[0].descriptorSize(); i++)
		{
			model1_descriptor->points[0].histogram[i] += model2_descriptor->points[0].histogram[i];
		}
	}
	float total = 0.0;
	for (int i = 0; i < model1_descriptor->points[0].descriptorSize(); i++)
	{
		model1_descriptor->points[0].histogram[i] /= 5;
		total += model1_descriptor->points[0].histogram[i];
	}
	cout << total << endl;
	cout << "ESF Time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;
	
	return model1_descriptor->points[0];
}

void ESFtest(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud , pcl::ESFSignature640 model)
{
	pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esf;
	pcl::PointCloud<pcl::ESFSignature640>::Ptr model1_descriptor(new pcl::PointCloud<pcl::ESFSignature640>);

	// ESF estimation object.

	esf.setInputCloud(cloud);
	esf.compute(*model1_descriptor);

	float dis = 0.0;
	for (int i = 0; i < model1_descriptor->points[0].descriptorSize(); i++)
	{
		float a = model.histogram[i];
		float b = model1_descriptor->points[0].histogram[i];
		dis += (a - b)*(a - b);
	}
	dis = sqrt(dis);

	cout << "ESF distance: " << dis << endl;
}

void show_plane_normal(pcl::visualization::PCLVisualizer& viewer, vector<Plane> planes, string id)
{
	for (int i = 0; i < 5; i++)
	{
		if (i != 1 && i != 2)
		{
			pcl::PointXYZ end(planes[i].box_center.x + planes[i].normal[0] * 0.5, planes[i].box_center.y + planes[i].normal[1] * 0.5, planes[i].box_center.z + planes[i].normal[2] * 0.5);
					viewer.addArrow(end, planes[i].box_center, 0.0, 0.0, 1.0, false, "arrow_z1" + id + to_string(i));
		}
		
	}
}

int main(int argc, char *argv[])
{
	//---------------------Model processing--------------------
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr sample_cloud_normal(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::io::loadPLYFile(string(argv[1]) + ".ply", *sample_cloud_normal);

	save_model model_file;
	registration reg;

	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	//viewer.addCoordinateSystem(0.3, "coordinate", 0);
	viewer.initCameraParameters();
	viewer.setBackgroundColor(255, 255, 255);
	
	vector<Plane> model_plane;
	Plane temp_plane;
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints(new pcl::PointCloud<pcl::PointXYZ>);
	vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> model_descriptors;
	pcl::ESFSignature640 model_esf;
	/*
	reg.find_object_plane(viewer, sample_cloud_normal, "1", model_plane, 0.035, 0.15);

	for (int i = 0; i < model_plane.size(); i++)
	{

		float angle = acos(Eigen::Vector3f::UnitY().dot(model_plane[i].normal));
		//cout << "A:" << angle *180 / M_PI << endl;
		if (angle * 180 / M_PI > 90.0)
			model_plane[i].normal = model_plane[i].normal * -1;
	}
	
	model_keypoints = Harris3D_key_point(viewer, sample_cloud_normal,"m"); //Extract keypoints
	model_descriptors = Kdescriptor(viewer, sample_cloud_normal, model_keypoints, "m"); //Calculate descriptors
	*/
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	
	pcl::copyPointCloud(*sample_cloud_normal, *cloud);

	//model_esf = ESFdescriptor(viewer, cloud);
	

	
	model_file.load_model_data(argv[1]);
	model_keypoints = model_file.load_model_keypoints();
	//model_descriptors = Kdescriptor(viewer, sample_cloud_normal, model_keypoints, "m"); //Calculate descriptors
	//model_descriptors = model_file.load_model_descriptors();
	model_plane = model_file.load_model_planes();
	//model_esf = model_file.load_model_ESF();
	
	//model_file.save_model_data(argv[1], model_keypoints, model_descriptors, model_plane, model_esf);

	//ESFtest(cloud, model_esf);

	show_plane_normal(viewer, model_plane, "m");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> harris_color_handler_m(model_keypoints, 255, 0, 0);
	viewer.addPointCloud(model_keypoints, harris_color_handler_m, "harris_model");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "harris_model");
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> color_handler_m(sample_cloud_normal, 150, 150, 150);
	viewer.addPointCloud<pcl::PointXYZRGBNormal>(sample_cloud_normal, color_handler_m,"model_cloud");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "model_cloud");

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	//---------------------Model processing--------------------

	return 0;
}