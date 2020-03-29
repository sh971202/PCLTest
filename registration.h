#include <iostream>
#include <time.h>
#include <string>
#include <vector>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/segmentation/extract_clusters.h>
#include "struct_type.h"

class registration
{
	public:
		//Euclidean_Cluster
		void Euclidean_Cluster(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
		pcl::PointCloud<pcl::PointXYZRGBNormal> get_object_cloud(int number);
		int get_object_number();

		//find planes on object
		void find_object_plane(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, std::string id, std::vector<Plane>& planes, float distrance_threshold, float size_threshold);

		//ICP
		double get_model_transformation(pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud);

	private:
		//Euclidean_Cluster variable
		std::vector<pcl::PointCloud<pcl::PointXYZRGBNormal>> object_cloud;
		int total_object_number;
};