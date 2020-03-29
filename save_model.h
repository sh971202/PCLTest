#include <iostream>
#include <string>
#include <pcl/point_types.h>
#include "./json/json.h"
#include "struct_type.h"

using namespace std;

class save_model {
	public:
		void save_model_data(string model_name, pcl::PointCloud<pcl::PointXYZ>::Ptr, vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr>, vector<Plane>, pcl::ESFSignature640);
		void load_model_data(string model_name);
		pcl::PointCloud<pcl::PointXYZ>::Ptr load_model_keypoints();
		vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> load_model_descriptors();
		vector<Plane> load_model_planes();
		pcl::ESFSignature640 load_model_ESF();
	private:
		int size;
		bool isopen = false;
		Json::Value root;

};
