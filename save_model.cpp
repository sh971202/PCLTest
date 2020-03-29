#include <iostream>
#include <string>
#include <vector>
#include <time.h>
#include <fstream>
#include <pcl/point_types.h>
#include "./json/json.h"
#include "save_model.h"

void save_model::save_model_data(string model_name, pcl::PointCloud<pcl::PointXYZ>::Ptr keypoints, vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> descriptors, vector<Plane> planes, pcl::ESFSignature640 esf_descriptor) {
	Json::Value root;
	Json::Value keypoint;
	size = keypoints->points.size();
	keypoint["size"].append(size);
	for (int i = 0; i < size; i++) {
		keypoint["x"].append(keypoints->points[i].x);
		keypoint["y"].append(keypoints->points[i].y);
		keypoint["z"].append(keypoints->points[i].z);
	}
	root["keypoints"] = Json::Value(keypoint);

	Json::Value descriptor;
	int descriptor_size = descriptors.size();
	descriptor["descriptor size"].append(descriptor_size);
	for (int i = 0; i < descriptor_size; i++) {
		size = descriptors[i]->points.size();
		descriptor["size"].append(size);
		for (int j = 0; j < size; j++) {
			descriptor["x"].append(descriptors[i]->points[j].x);
			descriptor["y"].append(descriptors[i]->points[j].y);
			descriptor["z"].append(descriptors[i]->points[j].z);
			//descriptor["rgb"].append(descriptors[i]->points[j].rgb);
			descriptor["normal_x"].append(descriptors[i]->points[j].normal[0]);
			descriptor["normal_y"].append(descriptors[i]->points[j].normal[1]);
			descriptor["normal_z"].append(descriptors[i]->points[j].normal[2]);
		}
	}
	root["descriptors"] = Json::Value(descriptor);

	Json::Value plane;
	size = planes.size();
	plane["size"].append(size);
	for (int i = 0; i < size; i++) {
		plane["box center_x"].append(planes[i].box_center.x);
		plane["box center_y"].append(planes[i].box_center.y);
		plane["box center_z"].append(planes[i].box_center.z);
		plane["height"].append(planes[i].height);
		plane["width"].append(planes[i].width);
		plane["normal_x"].append(planes[i].normal[0]);
		plane["normal_y"].append(planes[i].normal[1]);
		plane["normal_z"].append(planes[i].normal[2]);
	}
	root["planes"] = Json::Value(plane);

	Json::Value esf;
	size = 640;
	esf["size"].append(size);
	for (int i = 0; i < size; i++) {
		esf["value"].append(esf_descriptor.histogram[i]);

	}
	root["ESF"] = Json::Value(esf);

	ofstream os;
	string filename = model_name + ".json";
	os.open(filename);
	Json::StyledWriter sw;
	os << sw.write(root);
	os.close();
	cout << "save model file!" << endl;
}

void save_model::load_model_data(string model_name)
{
	Json::Reader reader;
	string filename = model_name + ".json";
	ifstream in(filename);
	if (!in.is_open()) {
		cout << "Open file error!" << endl;
	}
	if (reader.parse(in, root)) 
	{
		isopen = true;
		//cout << "Load file is done!" << endl;
	}
	in.close();

}


pcl::PointCloud<pcl::PointXYZ>::Ptr save_model::load_model_keypoints() 
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr model_keypoints(new pcl::PointCloud<pcl::PointXYZ>);
	if (isopen) {
		int number = root["keypoints"]["size"][0].asInt();
		model_keypoints->points.resize(number);
		for (int i = 0; i < number; i++) {
			model_keypoints->points[i].x = root["keypoints"]["x"][i].asFloat();
			model_keypoints->points[i].y = root["keypoints"]["y"][i].asFloat();
			model_keypoints->points[i].z = root["keypoints"]["z"][i].asFloat();
		}
	}
	else
	{
		cout << "There is no data!" << endl;
	}
	//cout << "sucess";
	return model_keypoints;
}

vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> save_model::load_model_descriptors() 
{
	vector<pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr> model_descriptors;
	if (isopen) {
		int number = root["descriptors"]["descriptor size"][0].asInt();
		int total = 0;
		for (int i = 0; i < number; i++) {
			pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
			int num = root["descriptors"]["size"][i].asInt();
			temp->points.resize(num);
			for (int j = 0; j < num; j++) {
				//model_descriptors[i]->points.resize(num);
				temp->points[j].x = root["descriptors"]["x"][total + j].asFloat();
				temp->points[j].y = root["descriptors"]["y"][total + j].asFloat();
				temp->points[j].z = root["descriptors"]["z"][total + j].asFloat();
				//temp->points[j].rgb = root["descriptors"]["rgb"][total + j].asFloat();
				temp->points[j].normal[0] = root["descriptors"]["normal_x"][total + j].asFloat();
				temp->points[j].normal[1] = root["descriptors"]["normal_y"][total + j].asFloat();
				temp->points[j].normal[2] = root["descriptors"]["normal_z"][total + j].asFloat();
			}
			total += num;
			model_descriptors.push_back(temp);
		}
	}
	else
	{
		cout << "There is no data!" << endl;
	}
	
	return model_descriptors;
}

vector<Plane> save_model::load_model_planes() 
{
	vector<Plane> model_planes;
	Plane temp;
	if (isopen) {
		int number = root["planes"]["size"][0].asInt();
		//cout << number;
		//model_planes.resize(number);
		for (int i = 0; i < number; i++) {
			temp.box_center.x = root["planes"]["box center_x"][i].asFloat();
			temp.box_center.y = root["planes"]["box center_y"][i].asFloat();
			temp.box_center.z = root["planes"]["box center_z"][i].asFloat();
			temp.normal[0] = root["planes"]["normal_x"][i].asFloat();
			temp.normal[1] = root["planes"]["normal_y"][i].asFloat();
			temp.normal[2] = root["planes"]["normal_z"][i].asFloat();
			temp.height = root["planes"]["height"][i].asFloat();
			temp.width = root["planes"]["width"][i].asFloat();
			model_planes.push_back(temp);
		}
	}
	else
	{
		cout << "There is no data!" << endl;
	}
	return model_planes;
}

pcl::ESFSignature640 save_model::load_model_ESF()
{
	pcl::ESFSignature640 temp;
	if (isopen) {
		int number = root["ESF"]["size"][0].asInt();
		
		for (int i = 0; i < number; i++) {
			temp.histogram[i] = root["ESF"]["value"][i].asFloat();
			
		}
	}
	else
	{
		cout << "There is no data!" << endl;
	}
	//cout << "sucess";
	return temp;
}