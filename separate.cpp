#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <pcl/io/io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/visualization/pcl_visualizer.h>
using namespace std;

int main(int argc,char *argv[])
{
	ifstream fin;
	ofstream fout;
	string line;

	pcl::PointCloud < pcl::PointXYZRGBNormal> ::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);

	if(argc==2)
		fin.open(argv[1]);
	else
	{
		cout<<"Please give your file name!!";
		return 1;
	}

	pcl::io::loadPLYFile(string(argv[1]) + ".ply", *cloud);

	pcl::visualization::PCLVisualizer viewer("Cloud Viewer");
	viewer.initCameraParameters();
	viewer.addPointCloud<pcl::PointXYZRGBNormal>(cloud);

	pcl::PointCloud<pcl::PointXYZRGBNormal> category[14];

	vector<int> indices;
	pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

	if (cloud->is_dense)
	{
		cout << "Dense!!" << endl;
	}
	else
	{
		cout << "Not Dense!!" << endl;
	}


	for (int n = 0; n < cloud->points.size(); n++)
	{
		float r, g, b;

		r = cloud->points[n].r;
		g = cloud->points[n].g;
		b = cloud->points[n].b;

		if (r == 0 && g == 128 && b == 128)
		{
			category[0].push_back(cloud->points[n]);
		}
		else if (r == 250 && g == 50 && b == 50)
		{
			category[1].push_back(cloud->points[n]);
		}
		else if (r == 102 && g == 0 && b == 204)
		{
			category[2].push_back(cloud->points[n]);
		}
		else if (r == 50 && g == 50 && b == 250)
		{
			category[3].push_back(cloud->points[n]);
		}
		else if (r == 220 && g == 220 && b == 220)
		{
			category[4].push_back(cloud->points[n]);
		}
		else if (r == 255 && g == 69 && b == 20)
		{
			category[5].push_back(cloud->points[n]);
		}
		else if (r == 255 && g == 20 && b == 127)
		{
			category[6].push_back(cloud->points[n]);
		}
		else if (r == 50 && g == 50 && b == 150)
		{
			category[7].push_back(cloud->points[n]);
		}
		else if (r == 222 && g == 180 && b == 140)
		{
			category[8].push_back(cloud->points[n]);
		}
		else if (r == 50 && g == 250 && b == 50)
		{
			category[9].push_back(cloud->points[n]);
		}
		else if (r == 255 && g == 215 && b == 0)
		{
			category[10].push_back(cloud->points[n]);
		}
		else if (r == 150 && g == 150 && b == 150)
		{
			category[11].push_back(cloud->points[n]);
		}
		else if (r == 0 && g == 255 && b == 255)
		{
			category[12].push_back(cloud->points[n]);
		}
		else if (r == 255 && g == 255 && b == 255)
		{
			category[13].push_back(cloud->points[n]);
		}
	}
	
	cout<<"Total_number"<< cloud->points.size() <<endl;
	
	for(int i=0;i<14;i++)
	{
		cout << "Class_" << i + 1 << "_count" << ": " << category[i].points.size() << endl;
	}

	pcl::PLYWriter writer;
	string file_path(argv[1]);

	for(int i=0;i<13;i++)
	{
		//writer.write(file_path+"_" + to_string(i+1) + ".ply", category[i], true, false);
	}

	cout << "Separate done!!" << endl;

	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return 0;
}
