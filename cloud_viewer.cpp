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

#include "pre_processing.h"
#include "registration.h"
#include "save_model.h"
#include "./json/save_json.h"


#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkInteractionStyle);
VTK_MODULE_INIT(vtkRenderingFreeType);
VTK_MODULE_INIT(vtkRenderingOpenGL);

using namespace std;

pcl::visualization::PCLVisualizer A("after");

//global variable
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr sample_cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_class(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr sample_cloud_normal(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

vector<Eigen::Matrix4f> T_list;

pre_processing process;
registration reg;
save_json test;

int user_data;
bool next_iteration = false;
    
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,void* nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		next_iteration = true;
}

void save_data()
{
	/*
	ofstream fout;

	fout.open(filename+".txt");

	fout << "Coefficients of Ground plane" << endl;
	pcl::ModelCoefficients coefficients=process.get_plane_coefficients();
	fout << coefficients.values[0] << " " << coefficients.values[1] << " " << coefficients.values[2] << " " << coefficients.values[3] << endl;

	fout << "Mass position of Ground plane" << endl;
	Eigen::Vector4f centroid=process.get_plane_centroid();
	fout << centroid[0] << " " << centroid[1] << " " << centroid[2] << endl;

	fout << "Center of Ground plane" << endl;
	pcl::PointXYZ plane_center = process.get_plane_center();
	fout << plane_center.x << " " << plane_center.y << " " << plane_center.z << endl;

	fout << "Size of Ground plane" << endl;
	pcl::PointXYZ plane_size = process.get_plane_size();
	fout << plane_size.x << " " << plane_size.y << " " << plane_size.z << endl;

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
	*/

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
		return ;
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
				cout << temp <<" ";
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

void load_models(pcl::visualization::PCLVisualizer& viewer,vector<Model> &sofa ,string class_name, int model_num)
{
	clock_t t;
	t = clock();
	for (int i = 0; i < model_num; i++)
	{
		Model temp;

		save_model model_file;
		pcl::io::loadPLYFile(class_name+to_string(i+1)+".ply", *temp.cloud);

		//pcl::copyPointCloud(*sofa[i].cloud, *sample_cloud); //copy original sample cloud

		model_file.load_model_data(class_name + to_string(i + 1));
		temp.keypoints = model_file.load_model_keypoints();
		temp.descriptors = model_file.load_model_descriptors();
		temp.planes = model_file.load_model_planes();
		temp.esf = model_file.load_model_ESF();
		temp.id = i + 1;
		
		/*
		pcl::visualization::PCLVisualizer M("M" + to_string(i + 1));
		M.addCoordinateSystem(0.3, "coordinate", 0);
		M.initCameraParameters();
		M.setBackgroundColor(0, 0, 0);

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> harris_color_handler_m(sofa[i].keypoints, 255, 0, 0);
		M.addPointCloud(sofa[i].keypoints, harris_color_handler_m, "harris_model" + to_string(i));
		M.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "harris_model" + to_string(i));
		M.addPointCloud(sample_cloud, "model_cloud" + to_string(i));
		*/

		sofa.push_back(temp);
	}
	cout << " Time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;
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

	cout << "After PCAfilter: "<< cloud_harris->size() << endl;

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

vector<Correspondence> Matching_descriptor(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoint, pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoint, vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> scene_descriptor, vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> model_decriptor,int match_num,
										vector<Plane> object_planes, vector<Plane> model_planes,string id)
{
	clock_t t;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGBNormal>), temp2(new pcl::PointCloud<pcl::PointXYZRGBNormal>), final_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	vector<Correspondence> C;
	vector<vector<int>> key_point_pair_list;

	pcl::KdTreeFLANN<pcl::PointXYZRGBNormal> kdtree;
	std::vector<int> pointIdxNKNSearch(1);
	std::vector<float> pointNKNSquaredDistance(1);

	t = clock();

	key_point_pair_list.resize(scene_descriptor.size());

	for (int i = 0; i < object_planes.size(); i++)
	{
		vector<int> index_s, index_m;
		float a, b, c, d;
		float angle_s = acos(object_planes[i].normal.dot(Eigen::Vector3f::UnitY())) / M_PI * 180;

		a = object_planes[i].normal[0];
		b = object_planes[i].normal[1];
		c = object_planes[i].normal[2];
		d = -(object_planes[i].normal[0] * object_planes[i].box_center.x + object_planes[i].normal[1] * object_planes[i].box_center.y + object_planes[i].normal[2] * object_planes[i].box_center.z);

		for (int p = 0; p < scene_descriptor.size(); p++)
		{
			float distance = abs(a * scene_keypoint->points[p].x + b * scene_keypoint->points[p].y + c * scene_keypoint->points[p].z + d);  //the distance that key_point_s to plane

			if (distance < 0.1) //the key_point is inliner if the distance is smaller than 10cm
			{
				index_s.push_back(p);
				//viewer.addSphere(scene_keypoint->points[p], 0.05, "r" + to_string(p) + "n"+id);
			}
		}

		for (int m = 0; m < model_planes.size(); m++)
		{
			float angle_m = acos(model_planes[m].normal.dot(Eigen::Vector3f::UnitY())) / M_PI * 180;

			// the angle between scene_plane and model_plane is smaller than 5 degree;
			if (abs(angle_m - angle_s) < 10)
			{
				a = model_planes[m].normal[0];
				b = model_planes[m].normal[1];
				c = model_planes[m].normal[2];
				d = -(model_planes[m].normal[0] * model_planes[m].box_center.x + model_planes[m].normal[1] * model_planes[m].box_center.y + model_planes[m].normal[2] * model_planes[m].box_center.z);

				for (int p = 0; p < model_decriptor.size(); p++)
				{
					float distance = abs(a * model_keypoint->points[p].x + b * model_keypoint->points[p].y + c * model_keypoint->points[p].z + d);  //the distance that key_point_m to plane

					if (distance < 0.1) //the key_point is inliner if the distance is smaller than 10cm
					{
						//check if index_m has the same index
						int op = 0;
						for (int k = 0; k < index_m.size(); k++)
						{
							if (index_m[k] == p)
							{
								op = 1;
								break;
							}
						}
						if (op == 0)
						{
							index_m.push_back(p);
						}
					}
				}
			}
		}
		//make a pair list which may be a correspondence
		for (int n = 0; n < index_s.size(); n++)
		{
			if (key_point_pair_list[index_s[n]].size() != 0)
			{
				for (int m = 0; m < index_m.size(); m++)
				{
					int op = 0;
					for (int a = 0; a < key_point_pair_list[index_s[n]].size(); a++)
					{
						if (key_point_pair_list[index_s[n]][a] == index_m[m])
						{
							op = 1;
							break;
						}
					}

					if (op == 0)
					{
						key_point_pair_list[index_s[n]].push_back(index_m[m]);
					}
				}
			}
			else
			{
				for (int m = 0; m < index_m.size(); m++)
				{
					key_point_pair_list[index_s[n]].push_back(index_m[m]);
				}
			}
		}
	}
	/*
	for (int j = 0; j < key_point_pair_list.size(); j++)
	{
		key_point_pair_list[j].clear();

		for (int i = 0; i < model_keypoint->points.size(); i++)
		{
			key_point_pair_list[j].push_back(i);
		}
	}
	*/
	
	for (int i = 0; i < key_point_pair_list.size(); i++)
	{
		cout << "[" << i << "]: ";

		for (int j = 0; j < key_point_pair_list[i].size(); j++)
		{
			cout << key_point_pair_list[i][j] << " ";
		}
		cout << endl;
	}

	int count = 0;
	for (int i = 0; i < key_point_pair_list.size(); i++)
	{
		vector<float> Min_E;
		vector<float> th;
		vector<int> Min_index;
		vector<Correspondence> temp_c;

		for (int m = 0; m < key_point_pair_list[i].size(); m++)
		{
			int j = key_point_pair_list[i][m]; // index of model_keypoints
			//matching rule
			if ((((model_keypoint->points[j].y / scene_keypoint->points[i].y) >= 1/1.5  && (model_keypoint->points[j].y / scene_keypoint->points[i].y) <= 1.5) || abs(model_keypoint->points[j].y - scene_keypoint->points[i].y) <= 0.1) )
			{
				count++;

				Correspondence cor;

				cor.m_index = j;
				cor.s_index = i;

				//model descriptor using kdtree to search
				kdtree.setInputCloud(model_decriptor[j]);

				Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

				//align scene keypoint and model keypoint for matching
				transformation(0, 3) = -scene_keypoint->points[i].x;
				transformation(1, 3) = -scene_keypoint->points[i].y;
				transformation(2, 3) = -scene_keypoint->points[i].z;

				pcl::transformPointCloud(*scene_descriptor[i], *temp, transformation);


				float temp_E=-1, temp_th;
				for (int k = 0; k < 36; k++)
				{
					Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
					transform_2.rotate(Eigen::AngleAxisf(M_PI / 18 * -k, Eigen::Vector3f::UnitY()));

					Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

					T = transform_2.matrix();

					T(0, 3) = model_keypoint->points[j].x;
					T(1, 3) = model_keypoint->points[j].y;
					T(2, 3) = model_keypoint->points[j].z;

					pcl::transformPointCloudWithNormals(*temp, *temp2, T);
					//pcl::transformPointCloud(*temp, *temp2, T);

					//two descriptor's Euclidean
					float E = 0.0;
					for (int p = 0; p < temp2->size(); p++)
					{
						if (kdtree.nearestKSearch(temp2->points[p], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
						{
							//cout << (1.0 + temp2->points[p].getNormalVector3fMap().dot(model_decriptor[j]->points[pointIdxNKNSearch[0]].getNormalVector3fMap())) / 2 << endl;
							//E = E + (pointNKNSquaredDistance[0])*(pointNKNSquaredDistance[0])*(pointNKNSquaredDistance[0])*(pointNKNSquaredDistance[0]);
							//E += 1.0-exp(-pointNKNSquaredDistance[0] * 50);  
							E += 0.8*(1.0 - exp(-pointNKNSquaredDistance[0] *50)) + 0.2*(1.0 - (1.0 + temp2->points[p].getNormalVector3fMap().dot(model_decriptor[j]->points[pointIdxNKNSearch[0]].getNormalVector3fMap())) / 2);
						}
					}
					//E = E * 10e10 / temp2->size();
					E /= temp2->size();
					//cout << "E:" << E << endl;
					cor.distance[k] = E;

					if (temp_E > E || temp_E==-1)
					{
						temp_E = E;
						temp_th = k;
						//pcl::copyPointCloud(*temp2, *final_cloud); //copy best model cloud
					}
				}
				//cout << "Min E:" << temp_E << endl;
				//cout << "Angle:" << temp_th << endl;
				//cout << "Scale: " << model_keypoint->points[j].y / scene_keypoint->points[i].y << endl;
				//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tc_handler(final_cloud, 255, 0, 255);
				//viewer.addPointCloud(final_cloud, tc_handler, "align" + to_string(i)+ to_string(j));

				//check if E is smaller than first match_num correspondence;
				int insert;
				for (insert = 0; insert < Min_E.size(); insert++)
				{
					if (Min_E[insert] > temp_E)
					{
						break;
					}
				}
				Min_E.insert(Min_E.begin() + insert, temp_E);
				Min_index.insert(Min_index.begin() + insert, j);
				th.insert(th.begin() + insert, temp_th);
				temp_c.insert(temp_c.begin() + insert, cor);
			  }
		}
		//cout << "size:" << Min_E.size() << endl;
		if(Min_E.size() > match_num)
		{
			Min_E.resize(match_num);
			Min_index.resize(match_num);
			th.resize(match_num);
			temp_c.resize(match_num);
		}
		for (int a = 0; a < match_num && a < Min_E.size(); a++)
		{
			C.push_back(temp_c[a]);
		}
		/*
		//get estimated transformation
		if (Min_index.size() > 0)
		{
			for (int c = 0; c < Min_index.size() && c <match_num; c++)
			{
				Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
				Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
				transform_2.rotate(Eigen::AngleAxisf(M_PI / 18 * (-th[c]), Eigen::Vector3f::UnitY()));

				float scale = scene_keypoint->points[i].y / model_keypoint->points[Min_index[c]].y;

				T(0, 0) = scale;
				T(1, 1) = scale;
				T(2, 2) = scale;

				T = transform_2.matrix() * T;

				T(0, 3) = scene_keypoint->points[i].x - (model_keypoint->points[Min_index[c]].x * T(0, 0) + model_keypoint->points[Min_index[c]].y * T(0, 1) + model_keypoint->points[Min_index[c]].z * T(0, 2));
				T(1, 3) = 0;
				T(2, 3) = scene_keypoint->points[i].z - (model_keypoint->points[Min_index[c]].x * T(2, 0) + model_keypoint->points[Min_index[c]].y * T(2, 1) + model_keypoint->points[Min_index[c]].z * T(2, 2));

				T_list.push_back(T);
			}
			
		}
		else
		{
			Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
			T_list.push_back(T);
		}
		*/
	}
	cout << "Matching descriptor time: " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;
	cout << "C size(old): " << C.size() << endl;
	cout << "Count: " << count << endl;
	for (int i = 0; i < C.size(); i++)
	{
		//viewer.addLine(scene_keypoint->points[C[i].s_index], model_keypoint->points[C[i].m_index], 0, 255, 0, "C" + to_string(i) + id );
	}
	/*
	t = clock();
	
	for (int i = 0; i < object_planes.size(); i++)
	{
		vector<int> index_s,index_m;
		float a, b, c, d;
		float angle_s = acos(object_planes[i].normal.dot(Eigen::Vector3f::UnitY())) / M_PI *180;

		a = object_planes[i].normal[0];
		b = object_planes[i].normal[1];
		c = object_planes[i].normal[2];
		d = -(object_planes[i].normal[0] * object_planes[i].box_center.x + object_planes[i].normal[1] * object_planes[i].box_center.y + object_planes[i].normal[2] * object_planes[i].box_center.z);

		for (int p = 0; p < scene_keypoint->size(); p++)
		{
			float distance = abs(a * scene_keypoint->points[p].x + b * scene_keypoint->points[p].y + c * scene_keypoint->points[p].z + d);  //the distance that key_point_s to plane

			if (distance < 0.05) //the key_point is inliner if the distance is smaller than 5cm
			{
				index_s.push_back(p);
				//viewer.addSphere(scene_keypoint->points[p], 0.05, "r" + to_string(p) + "n"+id);
			}
		}

		for (int m = 0; m < model_planes.size(); m++)
		{
			float angle_m = acos(model_planes[m].normal.dot(Eigen::Vector3f::UnitY())) / M_PI * 180;

			// the angle between scene_plane and model_plane is smaller than 10 degree;
			if (abs(angle_m - angle_s) < 10) 
			{
				a = model_planes[m].normal[0];
				b = model_planes[m].normal[1];
				c = model_planes[m].normal[2];
				d= -(model_planes[m].normal[0] * model_planes[m].box_center.x + model_planes[m].normal[1] * model_planes[m].box_center.y + model_planes[m].normal[2] * model_planes[m].box_center.z);

				for (int p = 0; p < model_keypoint->size(); p++)
				{
					float distance = abs(a * model_keypoint->points[p].x + b * model_keypoint->points[p].y + c * model_keypoint->points[p].z + d);  //the distance that key_point_m to plane

					if (distance < 0.05) //the key_point is inliner if the distance is smaller than 5cm
					{
						index_m.push_back(p);
					}
				}
			}
		}
		if (model_planes.size() != 0)
		{
			//filtering wrong correspondence
			for (int n = 0; n < C.size(); n++)
			{
				bool find_s = false, find_m =false;
				for (int s = 0; s < index_s.size(); s++)
				{
					if (C[n].s_index == index_s[s])
					{
						find_s = true;
						break;
					}
				}
				for (int m = 0; m < index_m.size(); m++)
				{
					if (C[n].m_index == index_m[m])
					{
						find_m = true;
						break;
					}
				}
				if (find_s == false && find_m == false)
				{
					continue;
				}
				else if (find_s == true && find_m == true)
				{
					C[n].find = 2;
				}
				else
				{
					if(C[n].find == 0)
						C[n].find = 1;
				}
			}
		}
	}
	for (vector<Correspondence>::iterator c = C.begin(); c != C.end();)
	{
		if (c->find == 1)
		{
			c = C.erase(c);
		}
		else
		{
			c++;
		}
	}
	cout << "Filtering correspondence time: " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;
	cout << "C size(new): " << C.size() << endl;
	*/
	for (int i = 0; i < C.size(); i++)
	{
		//A.addLine(scene_keypoint->points[C[i].s_index], model_keypoint->points[C[i].m_index], 0, 255, 0, "C" + to_string(i) + id);
	}
	
	return C;
}


float Dis(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
	float x_pos, y_pos, z_pos, distance;

	x_pos = (p1.x - p2.x) * (p1.x - p2.x);
	y_pos = (p1.y - p2.y) * (p1.y - p2.y);
	z_pos = (p1.z - p2.z) * (p1.z - p2.z);

	distance = sqrt(x_pos + y_pos + z_pos);

	return distance;
}


Trans_list Exhaustive_transformation(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoint, pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoint, vector<Correspondence> C,int match_num,int iteration)
{
	clock_t t;
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_keypoints(new pcl::PointCloud<pcl::PointXYZ>);
	vector<Correspondence> temp_c(C);
	Trans_list Transformation;

	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	vector<int> pointIdxRadiusSearch;
	vector<float> pointRadiusSquaredDistance;
	
	srand(time(NULL));  //use time as rand() seed
	t = clock();
	
	float max_q = 0.0;
	int max_inliner = 0;

	for (int index = 0; index < C.size(); index++)
	{
		for (int th = 0; th < 36; th++)
		{
			Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
			Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
			transform_2.rotate(Eigen::AngleAxisf(M_PI / 18 * (th), Eigen::Vector3f::UnitY()));

			float scale;
			if (scene_keypoint->points[C[index].s_index].y < 0.1 || model_keypoint->points[C[index].m_index].y < 0.1)
				scale = 1.0;
			else
				scale = scene_keypoint->points[C[index].s_index].y / model_keypoint->points[C[index].m_index].y;

			T(0, 0) = scale;
			T(1, 1) = scale;
			T(2, 2) = scale;

			T = transform_2.matrix() * T;

			T(0, 3) = scene_keypoint->points[C[index].s_index].x - (model_keypoint->points[C[index].m_index].x * T(0, 0) + model_keypoint->points[C[index].m_index].y * T(0, 1) + model_keypoint->points[C[index].m_index].z * T(0, 2));
			T(1, 3) = 0;
			T(2, 3) = scene_keypoint->points[C[index].s_index].z - (model_keypoint->points[C[index].m_index].x * T(2, 0) + model_keypoint->points[C[index].m_index].y * T(2, 1) + model_keypoint->points[C[index].m_index].z * T(2, 2));

			pcl::transformPointCloud(*model_keypoint, *temp_keypoints, T);

			Trans_list temp_T;

			temp_c.clear();

			kdtree.setInputCloud(temp_keypoints);

			float total_distance = 0.0;
			int num_count = 0;

			for (int s = 0; s < scene_keypoint->size(); s++)
			{
				
				float distance = -1.0;
				int index_d;

				//find minimum distance of each scene_keypoint between correspondences
				for (int j = 0; j < C.size(); j++)
				{
					if (C[j].s_index == s)
					{
						float temp_dis = Dis(scene_keypoint->points[s], temp_keypoints->points[C[j].m_index]);
						if (temp_dis < distance || distance == -1.0)
						{
							distance = temp_dis;
							index_d = j;  
						}
					}
				}

				if (distance != -1.0)
				{
					temp_T.quality += 0.7*(1.0-C[index_d].distance[th]) + 0.3*exp(-distance);
					//temp_T.quality += (1.0 - C[index_d].distance[th]);
				}

				
				/*
				if (kdtree.radiusSearch(scene_keypoint->points[s], 0.20f, pointIdxRadiusSearch, pointRadiusSquaredDistance))  // Euclidean Distance < 0.20m is neighborhood. (Radius)
				{
					
					float dis = -1.0;
					int index_d;
					for (int k = 0; k < pointIdxRadiusSearch.size(); k++)
					{
						for (int c = 0; c < C.size(); c++)
						{
							if (C[c].s_index == s && C[c].m_index == pointIdxRadiusSearch[k])
							{
								float phy_d = Dis(scene_keypoint->points[s], temp_keypoints->points[pointIdxRadiusSearch[k]]);
								if (dis == -1.0 || phy_d < dis)
								{
									dis = phy_d;
									index_d = c;
								}
							}
						}
					}
					
					if (dis != -1.0)
					{
						//cout << dis << endl;
						//cout << exp(-dis) << endl;
						//cout << C[index_d].distance[th] << endl;
						//cout << exp(-C[index_d].distance[th]) << endl;

						temp_c.push_back(C[index_d]);
						if (index_d == index)
						{
							//cout << C[index_d].distance[th] << endl;
							//temp_T.quality += 10.0 - C[index_d].distance[th];
							temp_T.quality += exp(-C[index_d].distance[th]) + exp(-dis);
						}
						else
						{
							//cout << C[index_d].distance[th] * dis * dis * 100 << endl;
							//temp_T.quality += 10.0 - C[index_d].distance[th] * dis * dis * 200;
							temp_T.quality += exp(-C[index_d].distance[th]) + exp(-dis);
						}
						temp_T.inliner++;
					}
				}
				*/
			}

			//cout << "Quality: " << temp_T.quality << endl;
			//cout << "Inlier: " << temp_T.inliner << endl;
			if (max_q < temp_T.quality)
			{
				max_q = temp_T.quality;
				max_inliner = temp_T.inliner;

				Transformation.T = T;
				Transformation.inliner = max_inliner;
				Transformation.C = C[index];
				Transformation.th = th;
				Transformation.quality = max_q;
			}

		}
	}
	cout << "Exhaustive to get transformation time: " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;
	cout << "Max_q: " << max_q << endl;
	cout << "Max_Inlier: " << max_inliner << endl; 
	cout << "Theta: " << Transformation.th << endl;

	return Transformation;
}

Trans_list Scale_transformation(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoint, pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoint, vector<Correspondence> C, Trans_list cadidate_T,int iteration)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_keypoints_m(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_keypoints_s(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr scale_keypoints_m(new pcl::PointCloud<pcl::PointXYZ>);

	Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();
	Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
	transform_2.rotate(Eigen::AngleAxisf(M_PI / 18 * (-cadidate_T.th), Eigen::Vector3f::UnitY()));

	Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

	pcl::transformPointCloud(*model_keypoint, *temp_keypoints_m, cadidate_T.T);

	translation(0, 3) = -temp_keypoints_m->points[cadidate_T.C.m_index].x;
	translation(1, 3) = -temp_keypoints_m->points[cadidate_T.C.m_index].y;
	translation(2, 3) = -temp_keypoints_m->points[cadidate_T.C.m_index].z;

	T = transform_2.matrix()*translation;
	  
	pcl::transformPointCloud(*temp_keypoints_m, *temp_keypoints_m, T);
	pcl::transformPointCloud(*scene_keypoint, *temp_keypoints_s, T);

	/*
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> harris_color_handler_s(temp_keypoints_s, 255, 255, 0);
	viewer.addPointCloud(temp_keypoints_s, harris_color_handler_s, "scale_s");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scale_s");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> harris_color_handler_m(temp_keypoints_m, 255, 0, 255);
	viewer.addPointCloud(temp_keypoints_m, harris_color_handler_m, "scale_m");
	viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scale_m");
	*/

	clock_t t;

	t = clock();

	//vector<float> phy_dis;
	float Max_distance = 0.0;
	float scale_x = 1.0, scale_z = 1.0;
	float first_total_distance = 0.0;
	int nc = 0;
	for (int s = 0; s < temp_keypoints_s->size(); s++)
	{
		float distance = -1.0;
		for (int j = 0; j < C.size(); j++)
		{

			if (C[j].s_index == s)
			{
				float temp_dis = Dis(temp_keypoints_s->points[C[j].s_index], temp_keypoints_m->points[C[j].m_index]);
				if (temp_dis < distance || distance == -1.0)
					distance = temp_dis;
			}
		}

		if (distance != -1.0)
		{
			first_total_distance += distance;
			nc++;
		}
	}

	if (nc != 0)
		Max_distance = first_total_distance / nc;
	
	//cout << "MAX_D: " << Max_distance << endl;
	//cout << nc << endl;

	int Max_index;

	for(int i = 0;i<iteration;i++)
	{
		int index;
		float temp_scale_x,temp_scale_z;
		index = rand() % C.size();

		if (temp_keypoints_s->points[C[index].s_index].x * temp_keypoints_m->points[C[index].m_index].x > 0 && abs(temp_keypoints_s->points[C[index].s_index].x) >0.1 && abs(temp_keypoints_m->points[C[index].m_index].x) > 0.1)
		{
			temp_scale_x = temp_keypoints_s->points[C[index].s_index].x / temp_keypoints_m->points[C[index].m_index].x;
		}
		else
		{
			temp_scale_x = 1.0;
		}
		if (temp_keypoints_s->points[C[index].s_index].z * temp_keypoints_m->points[C[index].m_index].z > 0 && abs(temp_keypoints_s->points[C[index].s_index].z) >0.1 && abs(temp_keypoints_m->points[C[index].m_index].z) > 0.1)
		{
			temp_scale_z = temp_keypoints_s->points[C[index].s_index].z / temp_keypoints_m->points[C[index].m_index].z;
		}
		else
		{
			temp_scale_z = 1.0;
		}

		Eigen::Matrix4f temp_T = Eigen::Matrix4f::Identity();

		temp_T(0, 0) = temp_scale_x;
		temp_T(2, 2) = temp_scale_z;

		pcl::transformPointCloud(*temp_keypoints_m, *scale_keypoints_m, temp_T); // pre-scale the model_keypoints to check every correspondence distance

		//phy_dis.clear();  // reset distance list

		float total_distance=0.0;
		int num_count = 0;

		for (int s = 0; s < temp_keypoints_s->size(); s++)
		{
			float distance=-1.0;

			//find minimum distance of each scene_keypoint between correspondences
			for (int j = 0; j < C.size(); j++)
			{
				if (C[j].s_index == s)
				{
					float temp_dis = Dis(temp_keypoints_s->points[C[j].s_index], scale_keypoints_m->points[C[j].m_index]);
					if (temp_dis < distance || distance == -1.0)
						distance = temp_dis;
				}
			}

			if (distance != -1.0)
			{
				total_distance += distance;
				num_count++;
			}
		}

		/*
		cout << "D: " << phy_dis.size() << endl;
		for (int j = 0; j < phy_dis.size(); j++)
		{
			cout << phy_dis[j] << endl;
		}
		for (int j = 0; j < phy_dis.size() / 4; j++)
		{
			total_distance += phy_dis[j];
		}
		*/
		//cout << "Total_distance: " << total_distance << endl;
		//cout << "Num: " << num_count << endl;

		if (num_count != 0)
			total_distance /= num_count;

		if (total_distance < Max_distance && num_count != 0)
		{
			Max_distance = total_distance;
			//cout << "MAX_D: " << Max_distance << endl;
			//cout<< num_count<<endl;
			scale_x = temp_scale_x;
			scale_z = temp_scale_z;
			Max_index = index;
		}

	}
	cout << "RANSAC to get scale transformation time: " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;
	cout << "Final_Scale_x: " << scale_x << endl;
	cout << "Final_Scale_z: " << scale_z << endl;

	//cout << "Sx: " << temp_keypoints_s->points[C[Max_index].s_index].x <<endl;
	//cout << "Mx: " << temp_keypoints_m->points[C[Max_index].m_index].x << endl;

	Eigen::Matrix4f scale_T = Eigen::Matrix4f::Identity();

	scale_T(0, 0) = scale_x;
	scale_T(2, 2) = scale_z;

	cadidate_T.T = T.inverse() * scale_T * T * cadidate_T.T;

	//viewer.addSphere(scene_keypoint->points[C[Max_index].s_index], 0.1, "S");
	//viewer.addSphere(model_keypoint->points[C[Max_index].m_index], 0.1, "M");

	return cadidate_T;

	//viewer.addSphere(scene_keypoint->points[C[Max_index].s_index], 0.3, "S");
	//viewer.addSphere(model_keypoint->points[C[Max_index].m_index], 0.3, "M");
}

void show_plane_normal(pcl::visualization::PCLVisualizer& viewer,vector<Plane> planes, string id)
{
	for (int i = 0; i < planes.size(); i++)
	{
		//cout << "Plane equation" << normals[i][0] << "x+" << normals[i][1] << "y+" << normals[i][2] << "z+" << -(normals[i][0]* box_centers[i].x + normals[i][1] * box_centers[i].y + normals[i][2] * box_centers[i].z) << endl;
		float angle = acos(Eigen::Vector3f::UnitY().dot(planes[i].normal));
		//cout << "A:" << angle * 180 / M_PI << endl;
		pcl::PointXYZ end(planes[i].box_center.x + planes[i].normal[0]* 0.5, planes[i].box_center.y + planes[i].normal[1]* 0.5, planes[i].box_center.z + planes[i].normal[2]*0.5);
		viewer.addArrow(end, planes[i].box_center, 0.0, 0.0, 1.0, false, "arrow_z1" + id + to_string(i));
	}
	//cout << endl;
}

void show_normals(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normal, string id)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
	clock_t t;

	t = clock();
	cout << "curvature estimation start!" << endl;
	//curvature estimation
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
	pcl::PrincipalCurvaturesEstimation<pcl::PointXYZ, pcl::Normal, pcl::PrincipalCurvatures> pc;
	//pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;

	pcl::PointCloud<pcl::PrincipalCurvatures>::Ptr cloud_curvatures(new pcl::PointCloud<pcl::PrincipalCurvatures>);
	pc.setInputCloud(cloud);
	pc.setInputNormals(normal);
	pc.setSearchMethod(tree);
	pc.setRadiusSearch(0.05);  //5cm
	//pc.setKSearch(10);
	pc.compute(*cloud_curvatures);
	cout << "Curvature estimation time: " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;

	for (int i = 0; i < 20;i++)
	{
		cout << cloud_curvatures->points[i].pc1 *  cloud_curvatures->points[i].pc2 << endl;
	}
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 255,255,255);
	//viewer.addPointCloud(cloud, single_color,"cloud" + id);

	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> temp_color(temp, 255, 0, 0);
	//viewer.addPointCloud(temp, temp_color, "temp" + id);

	//viewer.addPointCloudPrincipalCurvatures<pcl::PointXYZ, pcl::Normal>(cloud, normal, cloud_curvatures, 1, 1, "curvatures"+id);

	//viewer.addPointCloudNormals<pcl::PointXYZ,pcl::Normal>(cloud, normal,1, 0.02, "normal" + id);
}

void viewerOneOff (pcl::visualization::PCLVisualizer& viewer)
{
    viewer.setBackgroundColor (255, 255, 255);

	/*
	pcl::PolygonMesh mesh;
	pcl::io::loadOBJFile("./model/stol.obj", mesh);

	pcl::PointCloud<pcl::PointXYZ>::Ptr sample_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(mesh.cloud, *sample_cloud);
	viewer.addPointCloud(sample_cloud,"sample_cloude");
	*/
	//viewer.addPointCloud(sample_cloud, "sample_cloude");
    /*
	pcl::PointXYZ o;
	o.x = centroid[0];
    o.y = centroid[1];
    o.z = centroid[2];
    viewer.addSphere (o, 0.1, "sphere", 0);*/
	
}
    
void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);

    //FIXME: possible race condition here:
    user_data++;
}
    
vector<int> ESFtest(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud,vector<Model> M)
{
	pcl::ESFEstimation<pcl::PointXYZ, pcl::ESFSignature640> esf;
	pcl::PointCloud<pcl::ESFSignature640>::Ptr scene_descriptor(new pcl::PointCloud<pcl::ESFSignature640>);

	// ESF estimation object.
	esf.setInputCloud(cloud);
	esf.compute(*scene_descriptor);

	vector<float> Min_D;
	vector<int> index;
	clock_t t;

	t = clock();

	for (int k = 0; k < M.size(); k++)
	{

		float dis = 0.0;
		for (int i = 0; i < scene_descriptor->points[0].descriptorSize(); i++)
		{
			float a = scene_descriptor->points[0].histogram[i];
			float b = M[k].esf.histogram[i];
			dis += (a - b)*(a - b);
		}
		dis = sqrt(dis);

		int insert;
		for (insert = 0; insert < Min_D.size(); insert++)
		{
			if (Min_D[insert] > dis)
			{
				break;
			}
		}
		Min_D.insert(Min_D.begin() + insert, dis);
		index.insert(index.begin() + insert, k);

		cout << "ESF distance: " << dis << endl;
	}
	/*
	if (Min_D.size() > match_num)
	{
		Min_D.resize(match_num);
		index.resize(match_num);
	}
	*/
	for (int i = 0; i < index.size(); i++)
	{
		cout << "Model: " << index[i]  + 1 << endl;
	}

	cout << "ESF Time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;

	return index;
}

void Plane_modeling(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud,vector<Plane> object_planes, vector<Plane> model_planes)
{
	clock_t t;
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	std::vector<int> pointIdxNKNSearch(1);
	std::vector<float> pointNKNSquaredDistance(1);

	float Max_score=0.0;
	Eigen::Matrix4f best_T;

	t = clock();

	for (int th = 0; th < 36; th++)
	{
		float score = 0.0;
		Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
		Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
		transform_2.rotate(Eigen::AngleAxisf(M_PI / 18 * (th), Eigen::Vector3f::UnitY()));

		float scale;
		if (object_planes[0].box_center.y < 0.1 || model_planes[0].box_center.y < 0.1)
			scale = 1.0;
		else
			scale = object_planes[0].box_center.y / model_planes[0].box_center.y;

		T(0, 0) = scale;
		T(1, 1) = scale;
		T(2, 2) = scale;

		T = transform_2.matrix() * T;

		T(0, 3) = object_planes[0].box_center.x - (model_planes[0].box_center.x * T(0, 0) + model_planes[0].box_center.y * T(0, 1) + model_planes[0].box_center.z * T(0, 2));
		T(1, 3) = 0;
		T(2, 3) = object_planes[0].box_center.z - (model_planes[0].box_center.x * T(2, 0) + model_planes[0].box_center.y * T(2, 1) + model_planes[0].box_center.z * T(2, 2));

		pcl::transformPointCloud(*model_cloud, *temp_cloud, T);

		kdtree.setInputCloud(temp_cloud);

		float coverage = 0.0;

		for (int p = 0; p < cloud->points.size(); p++)
		{
			if (kdtree.nearestKSearch(cloud->points[p], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
			{
				if (pointNKNSquaredDistance[0] < 0.01) //1cm
				{
					coverage++;
				}

			}
		}

		score = coverage / cloud->points.size();

		kdtree.setInputCloud(cloud);

		coverage = 0.0;

		for (int p = 0; p < temp_cloud->points.size(); p++)
		{
			if (kdtree.nearestKSearch(temp_cloud->points[p], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
			{
				if (pointNKNSquaredDistance[0] < 0.005) //1cm
				{
					coverage++;
				}
			}
		}

		score = score + coverage / temp_cloud->points.size();

		if (score > Max_score)
		{
			Max_score = score;
			best_T = T;
		}

	}
	cout << "Score: " << Max_score << endl;
	cout << "GV time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;
}

Eigen::Matrix4f TV_modeling(pcl::visualization::PCLVisualizer& viewer, vector<Plane> object_planes, vector<Plane> model_planes)
{
	Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
	T(0, 3) = -model_planes[0].box_center.x;
	T(1, 3) = -model_planes[0].box_center.y;
	T(2, 3) = -model_planes[0].box_center.z;

	float angle_m = acos(model_planes[0].normal.dot(Eigen::Vector3f::UnitY())) / M_PI * 180;

	for (int i = 0; i < object_planes.size(); i++)
	{
		float angle_s = acos(object_planes[i].normal.dot(Eigen::Vector3f::UnitY())) / M_PI * 180;

		// the angle between scene_plane and model_plane is smaller than 10 degree;
		if (abs(angle_m - angle_s) < 10)
		{
			Eigen::Vector3f normal;

			normal[0] = object_planes[i].normal[0];
			normal[1] = 0;
			normal[2] = object_planes[i].normal[2];

			Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();

			//scale(0, 0) = object_planes[i].width / 1.25;
			scale(0, 0) = object_planes[i].width / 0.9;
			scale(1, 1) = object_planes[i].height / 0.73;

			T = scale * T;

			T = process.CreateRotateMatrix(model_planes[0].normal, normal) * T;

			Eigen::Matrix4f tanslation = Eigen::Matrix4f::Identity();

			tanslation(0, 3) = object_planes[i].box_center.x;
			tanslation(1, 3) = object_planes[i].box_center.y;
			tanslation(2, 3) = object_planes[i].box_center.z;

			T = tanslation * T;
		}
	}

	return T;
}

int main (int argc, char *argv[])
{
	clock_t t;

	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr sofa_class(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr sofa_class_after(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	t = clock();

	int op = 0;

	if (op == 1)
	{
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_color(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

		pcl::io::loadPLYFile(argv[1], *cloud_class);
		pcl::io::loadPLYFile("./image_process/123_color.ply", *cloud_color);

		cout << "Load point cloud done!!" << " Time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;

		process.remove_invalid_point(cloud_color, cloud_class);

		process.check(cloud_class);

		load_data(argv[2]);
		//---------------Transform point cloud to upward orientation------------------
		process.compute_ground_normal_transformation();
		pcl::transformPointCloudWithNormals(*cloud_color, *cloud_color, process.get_plane_transformation());  //cloud with normal
		pcl::transformPointCloudWithNormals(*cloud_class, *cloud_class, process.get_plane_transformation());  //cloud with normal
		//----------------------------------------------------------------------------

		//---------------Transform point cloud to forward orientation-------------
		process.compute_wall_normal_transformation();
		pcl::transformPointCloudWithNormals(*cloud_color, *cloud_color, process.get_wall_transformation());  //cloud with normal
		pcl::transformPointCloudWithNormals(*cloud_class, *cloud_class, process.get_wall_transformation());  //cloud with normal
		//----------------------------------------------------------------------------
		
		t = clock();

		//filtering out wrong label point
		process.filter_wrong_point(cloud_color, cloud_class);
		
		
		for (int n = 0; n < cloud_class->points.size(); n++)
		{
			if (process.class_mapping(cloud_class->points[n]) == 5 || process.class_mapping(cloud_class->points[n]) == 9)
			{
				sofa_class->push_back(cloud_class->points[n]);
			}
		}
		
		cout << "class point:" << sofa_class->points.size() << endl;

		cout << "After remove wall and floor point:" << cloud_color->points.size() << endl;

		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

		*temp_cloud = process.refine_probability(cloud_color, cloud_class);

		for (int n = 0; n < temp_cloud->points.size(); n++)
		{
			if (process.class_mapping(temp_cloud->points[n]) == 5 || process.class_mapping(temp_cloud->points[n]) == 9)
			{
				sofa_class_after->push_back(temp_cloud->points[n]);
			}
		}
		cout << "class_after point:" << sofa_class_after->points.size() << endl;

		cout << "Pre-processing Time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;

		pcl::PLYWriter writer;
		//writer.write("./CSIE4-2/furniture_class.ply", *sofa_class, true, false);
	}
	else
	{
		pcl::io::loadPLYFile(argv[1], *sofa_class_after);
		load_data(argv[2]);
	}
	
	//blocks until the cloud is actually rendered
	//use the following functions to get access to the underlying more advanced/powerful
	//PCLVisualizer
	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.registerKeyboardCallback(&keyboardEventOccurred, (void*)NULL); //add keyboard event
	//viewer.addCoordinateSystem(0.3, "coordinate", 0);
	viewer.initCameraParameters();
	//This will only get called once
	viewerOneOff(viewer);

	viewer.addPointCloud<pcl::PointXYZRGBNormal>(sofa_class);
	/*
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_table(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	pcl::io::loadPLYFile("./CSIE2-2/table_class_after.ply", *cloud_table);

	*sofa_class_after = *sofa_class_after + *cloud_table;
	*/
	//
	A.addCoordinateSystem(0.3, "coordinate", 0);
	A.initCameraParameters();
	viewerOneOff(A);
	A.addPointCloud<pcl::PointXYZRGBNormal>(sofa_class_after);
	//
	/*
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}
	*/
	//---------------------Model processing--------------------
	vector<Model> sofa;

	//load_models(viewer, sofa, "./Model_pointcloud(modify)/table/table", 13);

	/*
	Model temp;
	save_model model_file;
	pcl::io::loadPLYFile("./Model_pointcloud(modify)/pc/pc1.ply", *temp.cloud);
	
	model_file.load_model_data("./Model_pointcloud(modify)/pc/pc1");
	temp.keypoints = model_file.load_model_keypoints();
	temp.descriptors = model_file.load_model_descriptors();
	temp.planes = model_file.load_model_planes();
	temp.esf = model_file.load_model_ESF();
	temp.id = 13;
	
	sofa.push_back(temp);
	*/
	//-----------------Model processing------------------
	//-----------------semantic modeling-----------------
	vector<Eigen::Matrix4f> T_list;
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> retrived_clouds;
	vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> retrived_keypoints;
	int count = 0;
	
	pcl::console::print_highlight("Downsampling...\n");
	pcl::VoxelGrid<pcl::PointXYZRGBNormal> grid;
	float leaf = 0.03f;
	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(sofa_class_after);
	grid.filter(*sofa_class_after);
	cout << "Scene points:" << sofa_class_after->size() << endl;

	reg.Euclidean_Cluster(sofa_class_after);

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	/*for (int n = 0; n < reg.get_object_number(); n++)
	{
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		pcl::PLYWriter writer;

		*cloud_cluster = reg.get_object_cloud(n); // get each object

		writer.write("./CSIE3/furniture_object_"+to_string(n)+".ply", *cloud_cluster, true, false);
	}
	return 0;*/

	clock_t modeling_t;

	modeling_t = clock();

	for (int n = 1; n < 2; n++)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr e_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr m_cloud(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints(new pcl::PointCloud<pcl::PointXYZ>);
		pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
		vector<Plane> object_plane;

		*cloud_cluster = reg.get_object_cloud(n); // get each object
		
		pcl::copyPointCloud(*cloud_cluster, *e_cloud); //copy original cloud

		reg.find_object_plane(viewer, cloud_cluster, to_string(count), object_plane,0.02,0.1);

		for (int i = 0; i < object_plane.size(); i++)
		{
			float angle = acos(Eigen::Vector3f::UnitY().dot(object_plane[i].normal));

			if (angle * 180 / M_PI > 90.0)
				object_plane[i].normal = object_plane[i].normal * -1;
		}
		
		//show_plane_normal(viewer, object_plane, to_string(n));
		//show_plane_normal(viewer, model_plane, "m"+to_string(n));
		/*
		ESFtest(viewer, e_cloud);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tc_handler(e_cloud, 100 * n, 100* n, 255);
		viewer.addPointCloud(e_cloud, tc_handler, "secne_cloud1" + to_string(n));
		continue;
		*/

		// Extract keypoints and Calculate descriptors from objects
		//-----------------Harris 3D------------------
		pcl::PointCloud<pcl::PointXYZ>::Ptr scene_keypoints(new pcl::PointCloud<pcl::PointXYZ>);
		vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> scene_descriptors;

		scene_keypoints = Harris3D_key_point(viewer, cloud_cluster, object_plane,to_string(count));

		scene_descriptors = Kdescriptor(viewer, cloud_cluster, scene_keypoints, to_string(count));
		//-----------------Harris 3D-------------------

		
		//viewer.addPointCloud(e_cloud, "secne_cloud" + to_string(count + n));

		//PCA_visualize(viewer, scene_descriptors, scene_keypoints, to_string(n));
		//continue;
		//ESFtest(viewer, e_cloud);

		//-----------------Matching-------------------
		int id = 0;
		float MAX_Score = 0.0;
		Eigen::Matrix4f best_T = Eigen::Matrix4f::Identity();
		vector<int> index_m;
		vector<Model> class_models;

		//index_m = ESFtest(viewer, e_cloud,sofa);

		class_models = sofa;
		/*
		for (int m = 0; m < index_m.size(); m++)
		{
			class_models.push_back(sofa[index_m[m]]);
		}
		*/
		int model_count = 0;
		
		clock_t modeling_t;

		modeling_t = clock();

		vector<Correspondence> cors;
		/*
		//Plane modeling method
		if (scene_keypoints->size() == 0)
		{
			for (int i = 0; i < class_models.size() && model_count != 3; i++)
			{
				clock_t t;
				pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
				std::vector<int> pointIdxNKNSearch(1);
				std::vector<float> pointNKNSquaredDistance(1);

				float temp_Max_score = 0.0;
				Eigen::Matrix4f temp_best_T;

				t = clock();

				pcl::copyPointCloud(*class_models[i].cloud, *temp_cloud);

				for (int th = 0; th < 36; th++)
				{
					float score = 0.0;
					pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud2(new pcl::PointCloud<pcl::PointXYZ>);
					Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
					Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
					transform_2.rotate(Eigen::AngleAxisf(M_PI / 18 * (th), Eigen::Vector3f::UnitY()));

					float scale;
					if (object_plane[0].box_center.y < 0.1 || class_models[i].planes[0].box_center.y < 0.1)
						scale = 1.0;
					else
						scale = object_plane[0].box_center.y / class_models[i].planes[0].box_center.y;

					T(0, 0) = 1;
					T(1, 1) = scale;
					T(2, 2) = 1;

					T = transform_2.matrix() * T;

					T(0, 3) = object_plane[0].box_center.x - (class_models[i].planes[0].box_center.x * T(0, 0) + class_models[i].planes[0].box_center.y * T(0, 1) + class_models[i].planes[0].box_center.z * T(0, 2));
					T(1, 3) = 0;
					T(2, 3) = object_plane[0].box_center.z - (class_models[i].planes[0].box_center.x * T(2, 0) + class_models[i].planes[0].box_center.y * T(2, 1) + class_models[i].planes[0].box_center.z * T(2, 2));

					pcl::transformPointCloud(*temp_cloud, *temp_cloud2, T);

					kdtree.setInputCloud(temp_cloud2);

					float coverage = 0.0;

					for (int p = 0; p < e_cloud->points.size(); p++)
					{
						if (kdtree.nearestKSearch(e_cloud->points[p], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
						{
							if (pointNKNSquaredDistance[0] < 0.01) //1cm
							{
								coverage++;
							}

						}
					}

					score = coverage / e_cloud->points.size();

					kdtree.setInputCloud(e_cloud);

					coverage = 0.0;

					for (int p = 0; p < temp_cloud2->points.size(); p++)
					{
						if (kdtree.nearestKSearch(temp_cloud2->points[p], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
						{
							if (pointNKNSquaredDistance[0] < 0.005) //0.5cm
							{
								coverage++;
							}
						}
					}

					score = score + coverage / temp_cloud2->points.size();

					if (score > temp_Max_score)
					{
						temp_Max_score = score;
						temp_best_T = T;
					}

				}
				cout << "Score: " << temp_Max_score << endl;
				cout << "GV time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl << endl;

				if (temp_Max_score > MAX_Score)
				{
					MAX_Score = temp_Max_score;
					best_T = temp_best_T;
					id = i;
				}
				model_count++;
			}
		}
		//Keypoint modeling method
		else*/
		{
			for (int i = 0; i < class_models.size() ; i++)
			{
				//vector<Correspondence> cors;
				Trans_list final_T;

				vector<Eigen::Matrix4f> temp_T;
				vector<float> score;

				//best_T = TV_modeling(viewer, object_plane, class_models[i].planes);
				
				cors = Matching_descriptor(viewer, scene_keypoints, class_models[i].keypoints, scene_descriptors, class_models[i].descriptors, 4, object_plane, class_models[i].planes, to_string(count));

				if (cors.size() == 0)
				{

					continue;
				}

				Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

				final_T = Exhaustive_transformation(viewer, scene_keypoints, class_models[i].keypoints, cors, 4, 5000);
				//Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
				//T = final_T.T;

				final_T = Scale_transformation(viewer, scene_keypoints, class_models[i].keypoints, cors, final_T, 500);

				for (int k = 0; k < 3; k++)
				{
					float temp_score = 0.0;

					pcl::PointCloud<pcl::PointXYZ>::Ptr temp_keypoints(new pcl::PointCloud<pcl::PointXYZ>);

					pcl::transformPointCloud(*class_models[i].keypoints, *temp_keypoints, final_T.T);

					Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();
					Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
					//transform_2.rotate(Eigen::AngleAxisf(M_PI / 18 * (-final_T.th), Eigen::Vector3f::UnitY()));

					translation(0, 3) = -temp_keypoints->points[final_T.C.m_index].x;
					translation(1, 3) = -temp_keypoints->points[final_T.C.m_index].y;
					translation(2, 3) = -temp_keypoints->points[final_T.C.m_index].z;

					transform_2.rotate(Eigen::AngleAxisf(M_PI / 18 * (k - 1), Eigen::Vector3f::UnitY()));

					T = translation.inverse() * transform_2.matrix() * translation  * final_T.T;

					//final_T = Exhaustive_transformation(viewer, scene_keypoints, temp_keypoints, cors, 4, 5000);

					//T = transform_2.matrix()*T;

					pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
					pcl::copyPointCloud(*class_models[i].cloud, *temp_cloud);
					pcl::transformPointCloud(*temp_cloud, *temp_cloud, T);

					
					pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
					std::vector<int> pointIdxNKNSearch(1);
					std::vector<float> pointNKNSquaredDistance(1);

					kdtree.setInputCloud(temp_cloud);

					float coverage = 0.0;

					t = clock();
					for (int p = 0; p < e_cloud->points.size(); p++)
					{
						if (kdtree.nearestKSearch(e_cloud->points[p], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
						{
							if (pointNKNSquaredDistance[0] < 0.01) //1cm
							{
								coverage++;
							}

						}
					}

					temp_score = coverage / e_cloud->points.size();

					kdtree.setInputCloud(e_cloud);

					coverage = 0.0;

					for (int p = 0; p < temp_cloud->points.size(); p++)
					{
						if (kdtree.nearestKSearch(temp_cloud->points[p], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
						{
							if (pointNKNSquaredDistance[0] < 0.01) //0.5cm
							{
								coverage++;
							}
						}
					}

					temp_score = temp_score + coverage / temp_cloud->points.size();

					cout << "Score: " << temp_score << endl;
					cout << "GV time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;
					
					//temp_score = reg.get_model_transformation(temp_cloud, e_cloud); //ICP
					//T = final_T.T *T;

					cout << "T Matrix: " << endl;
					cout << T << endl << endl;

					score.push_back(temp_score);
					temp_T.push_back(T);

				}
				
				if (score[0] >= score[2] && (score[0] - score[1]) > 0.1)
				{
					if (score[0] > MAX_Score)
					{
						MAX_Score = score[0];
						best_T = temp_T[0];
						id = i;
					}
				}
				else if (score[0] < score[2] && (score[2] - score[1]) > 0.1)
				{
					if (score[2] > MAX_Score)
					{
						MAX_Score = score[2];
						best_T = temp_T[2];
						id = i;
					}
				}
				else
				{
					if (score[1] > MAX_Score)
					{
						MAX_Score = score[1];
						best_T = temp_T[1];
						id = i;
					}
				}
				
				model_count++;
				
			}
			
		}

		cout << endl << "Modeling time : " << float((clock() - modeling_t)) / CLOCKS_PER_SEC << " s" << endl;

		//-----------------Matching------------------- 
		pcl::copyPointCloud(*class_models[id].cloud, *m_cloud); //copy original sample cloud
		model_keypoints = class_models[id].keypoints;

		T_list.push_back(best_T);
		retrived_clouds.push_back(m_cloud);
		retrived_keypoints.push_back(model_keypoints);
		/*
		Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
		transform_2.rotate(Eigen::AngleAxisf(M_PI / 18 * (-9), Eigen::Vector3f::UnitY()));

		pcl::transformPointCloud(*m_cloud, *m_cloud, transform_2.matrix());
		pcl::transformPointCloud(*model_keypoints, *model_keypoints, transform_2.matrix());
		
		Eigen::Matrix4f translation = Eigen::Matrix4f::Identity();

		translation(0, 3) = 0.6;
		translation(1, 3) = 0;
		translation(2, 3) = -1.2;

		pcl::transformPointCloud(*m_cloud, *m_cloud, translation);
		pcl::transformPointCloud(*model_keypoints, *model_keypoints, translation);
		
		for (int i = 0; i < cors.size(); i++)
		{
			//if(cors[i].s_index == 0)
				viewer.addLine(scene_keypoints->points[cors[i].s_index], model_keypoints->points[cors[i].m_index], 0, 255, 0, "C" + to_string(i));
		}
		*/
		//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tc(e_cloud, 0, 50 * (n+1), 50 * n);
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tc(e_cloud, 255, 69, 20);
		viewer.addPointCloud(e_cloud, tc, "secne_cloud" + to_string(count));
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "secne_cloud" + to_string(count));
		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tc_handler(m_cloud, 0, 0, 255);
		viewer.addPointCloud(m_cloud, tc_handler, "model_cloud" + to_string(count));
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "model_cloud" + to_string(count));



		/*for (int i = 0; i < scene_descriptors.size(); i++)
		{
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> temp_color_handler(scene_descriptors[i], 0, 255, 0);
			viewer.addPointCloud(scene_descriptors[i], temp_color_handler, "kdescriptor" + to_string(i));
			viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "kdescriptor" + to_string(i));
		}
		*/

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> harris_color_handler_s(scene_keypoints, 255, 0, 0);
		viewer.addPointCloud(scene_keypoints, harris_color_handler_s, "harris_scene" + to_string(count));
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "harris_scene" + to_string(count));

		pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> harris_color_handler_m(model_keypoints, 255, 0, 0);
		viewer.addPointCloud(model_keypoints, harris_color_handler_m, "harris_model" + to_string(count));
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 8, "harris_model" + to_string(count));


		//viewer.addSphere(scene_keypoints->points[0],0.01,"sphere"+ to_string(count));

		test.save_obj_data("tv", class_models[id].id, T_list[count].data());
		//model_file.save_model_data("sofa", scene_keypoints, scene_descriptors, object_plane);
		count++;
	}

	save_data();
	test.save_file("output");
	
    while (!viewer.wasStopped())
    {
		viewer.spinOnce();
		// The user pressed "space" :
		if (next_iteration)
		{
			
			for (int n = 0; n < T_list.size(); n++)
			{
				pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
				pcl::PointCloud<pcl::PointXYZ>::Ptr temp_keypoints(new pcl::PointCloud<pcl::PointXYZ>);

				pcl::transformPointCloud(*retrived_clouds[n], *temp_cloud, T_list[n]);
				pcl::transformPointCloud(*retrived_keypoints[n], *temp_keypoints, T_list[n]);
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tc_handler(temp_cloud, 0, 0, 255);
				viewer.updatePointCloud(temp_cloud, tc_handler, "model_cloud" + to_string(n));
				pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> harris_color(temp_keypoints, 255, 0, 0);
				viewer.updatePointCloud(temp_keypoints, harris_color, "harris_model" + to_string(n));
				viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "harris_model" + to_string(n));
			}
			next_iteration = false;
		}
    }
    return 0;
}
