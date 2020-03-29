#include <iostream>
#include <string>
#include <time.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include "struct_type.h"

class pre_processing 
{
	public:
		//plane function
		void processing_ground_plane(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
		void compute_ground_normal_transformation();
		pcl::PointCloud<pcl::PointXYZ> get_plane_cloud();
		pcl::ModelCoefficients get_plane_coefficients();
		pcl::PointXYZ get_plane_center();
		pcl::PointXYZ get_plane_size();
		Eigen::Vector4f get_plane_centroid();
		Eigen::Matrix4f get_plane_transformation();
		void set_plane_coefficients(pcl::ModelCoefficients coefficients);
		void set_plane_centroid(Eigen::Vector4f centroid);
		void set_plane_center(pcl::PointXYZ center);
		void set_plane_size(pcl::PointXYZ size);

		//wall function
		void processing_wall(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);
		void compute_wall_normal_transformation();
		pcl::PointCloud<pcl::PointXYZ> get_walls_cloud(int number);
		pcl::PointXYZ get_walls_coefficients(int number);
		pcl::PointXYZ get_walls_size(int number);
		pcl::PointXYZ get_walls_center(int number);
		int get_walls_number();
		Eigen::Matrix4f get_wall_transformation();
		void set_walls_center(pcl::PointXYZ center, int number);
		void set_walls_coefficients(pcl::PointXYZ coefficients, int number);
		void set_walls_size(pcl::PointXYZ size, int number);
		void set_walls_number(int number);

		//compute rotation matrix of two vectors
		Eigen::Matrix4f CreateRotateMatrix(Eigen::Vector3f before, Eigen::Vector3f after);
		
		//remove invalid point from original point cloud 
		void remove_invalid_point(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_color, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_class);

		//check if there is any invalid point or not
		void check(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud);

		//get the class index of the point
		int class_mapping(pcl::PointXYZRGBNormal point);

		//fliter out the wrong points on the wall and ground
		void filter_wrong_point(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_class);

		//refine the probability of the classes
		pcl::PointCloud<pcl::PointXYZRGBNormal> refine_probability(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_color, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_class);

	private:
		//plane variable
		pcl::ModelCoefficients plane_coefficients;
		pcl::PointCloud<pcl::PointXYZ> plane_cloud;
		pcl::PointXYZ plane_size,plane_center;
		Eigen::Vector4f plane_centroid;
		Eigen::Matrix4f plane_transformation;


		//wall variable
		pcl::PointCloud<pcl::PointXYZ> walls_cloud[4];
		pcl::PointXYZ wall_center[4], wall_coefficients[4], wall_size[4];
		int wall_number;
		Eigen::Matrix4f wall_transformation;
		
};