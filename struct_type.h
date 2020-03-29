#include <vector>
#include <string>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace std;

#ifndef TYPE_H
#define TYPE_H

typedef struct Correspondence {
	int s_index;
	int m_index;
	float distance[36];
	int find = 0;
}Correspondence;

typedef struct Trans_list {
	Eigen::Matrix4f T;
	Correspondence C;
	int th;
	float quality = 0.0;
	int inliner = 0;
}Trans_list;

typedef struct Plane {
	Eigen::Vector3f normal;
	pcl::PointXYZ box_center;
	float height = 0.0;
	float width = 0.0;
}Plane;

typedef struct Model
{
	string category;
	int id;
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud = pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
	vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> descriptors;
	vector<Plane> planes;
	pcl::ESFSignature640 esf;

}Model;

#endif