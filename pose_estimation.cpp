#include <math.h>
#include<iostream>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/gasd.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_plotter.h>
#include <boost/thread/thread.hpp>

typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> ColorHandlerT;

// Align a rigid object to a scene with clutter and occlusions
int main(int argc, char **argv)
{
	pcl::PLYWriter writer;
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr object(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr scene(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pose(new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr icp_object(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr icp_scene(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr icp_pose(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
	
	
	pcl::io::loadPLYFile<pcl::PointXYZRGBA>("./pc1.ply", *object);
	pcl::io::loadPLYFile<pcl::PointXYZRGBA>("./cloud-1.ply", *scene);
	pcl::io::loadPLYFile<pcl::PointXYZRGBA>("./cofusion/cofusion_data/box6/8/cloud-1.ply", *pose);
	pcl::io::loadPLYFile<pcl::PointXYZRGBNormal>("./pc1.ply", *icp_object);
	pcl::io::loadPLYFile<pcl::PointXYZRGBNormal>("./cloud-1.ply", *icp_scene);
	pcl::io::loadPLYFile<pcl::PointXYZRGBNormal>("./cofusion/cofusion_data/box6/8/cloud-1.ply", *icp_pose);
	writer.write("./gasd_object.ply", *object);

	Eigen::Matrix4f pc_trans;
	pc_trans << 0.9069785475730896,
		0.0,
		0.0,
		0.0,
		0.0,
		0.9069785475730896,
		0.0,
		0.0,
		0.0,
		0.0,
		0.95213085412979126,
		0.0,
		-0.63535857200622559,
		0.0,
		1.1206178665161133,
		1.0;
	pcl::transformPointCloud(*object, *object, pc_trans);
	pcl::transformPointCloud(*icp_object, *icp_object, pc_trans);
	//writer.write("./gasd_object.ply", *object);

	// Downsample
	pcl::console::print_highlight("Downsampling...\n");

	clock_t t;
	t = clock();
	pcl::VoxelGrid<pcl::PointXYZRGBA> grid;
	const float leaf = 0.005f;
	grid.setLeafSize(leaf, leaf, leaf);
	grid.setInputCloud(object);
	grid.filter(*object);
	grid.setInputCloud(scene);
	grid.filter(*scene);
	grid.setInputCloud(pose);
	grid.filter(*pose);
	
	/*
	pcl::VoxelGrid<pcl::PointXYZRGBNormal> icp_grid;
	icp_grid.setLeafSize(leaf, leaf, leaf);
	icp_grid.setInputCloud(icp_object);
	icp_grid.filter(*icp_object);
	icp_grid.setInputCloud(icp_scene);
	icp_grid.filter(*icp_scene);
	icp_grid.setInputCloud(icp_pose);
	icp_grid.filter(*icp_pose);
	*/
	cout << "Downsampling... done!!" << " Time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;
	
	// Estimate features
	pcl::console::print_highlight("Estimating features...\n");
	pcl::GASDColorEstimation<pcl::PointXYZRGBA, pcl::GASDSignature984> gasd;
	gasd.setInputCloud(object);
	pcl::PointCloud<pcl::GASDSignature984> descriptor;
	gasd.compute(descriptor);
	Eigen::Matrix4f object_trans = gasd.getTransform();
	//pcl::transformPointCloud(*icp_object, *icp_object, object_trans);
	gasd.setInputCloud(scene);
	gasd.compute(descriptor);
	Eigen::Matrix4f scene_trans = gasd.getTransform();
	scene_trans = object_trans.inverse().eval() * scene_trans;
	pcl::transformPointCloud(*icp_scene, *icp_scene, scene_trans);
	pcl::transformPointCloud(*icp_pose, *icp_pose, scene_trans);
	
	pcl::console::print_highlight("ICP...\n");
	pcl::IterativeClosestPointWithNormals<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> icp;
	icp.setInputSource(icp_scene);
	icp.setInputTarget(icp_object);
	icp.setMaxCorrespondenceDistance(100);
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(0.001);
	icp.setMaximumIterations(100);
	icp.align(*icp_scene);
	//icp.align(*icp_pose);

	if (icp.hasConverged())
	{
		// Print results
		printf("\n");
		Eigen::Matrix4f transformation = icp.getFinalTransformation();
		//pcl::transformPointCloud(*icp_scene, *icp_scene, transformation);
		pcl::transformPointCloud(*icp_pose, *icp_pose, transformation);
		pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1), transformation(0, 2));
		pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1), transformation(1, 2));
		pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1), transformation(2, 2));
		pcl::console::print_info("\n");
		pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3), transformation(2, 3));
		pcl::console::print_info("\n");

		pcl::visualization::PCLVisualizer visu("Alignment");
		visu.addPointCloud(icp_scene, ColorHandlerT(icp_scene, 0.0, 255.0, 0.0), "scene");
		visu.addPointCloud(icp_object, ColorHandlerT(icp_object, 0.0, 0.0, 255.0), "object_aligned");
		visu.addPointCloud(icp_pose, ColorHandlerT(icp_pose, 0.0, 255.0, 255.0), "object_move");
		writer.write("./gasd_scene.ply", *icp_scene);
		writer.write("./gasd_object.ply", *icp_pose);
		visu.spin();
	}

	/*
	// Estimate features
	pcl::console::print_highlight("Estimating features...\n");
	gasd.setInputCloud(pose);
	gasd.compute(descriptor);
	Eigen::Matrix4f pose_trans = gasd.getTransform(); 
	gasd.setInputCloud(object);
	gasd.compute(descriptor);
	object_trans = gasd.getTransform();
	//pcl::transformPointCloud(*icp_object, *icp_object, object_trans);
	object_trans = pose_trans.inverse().eval();
	pcl::transformPointCloud(*icp_object, *icp_object, object_trans);
	writer.write("./gasd_pose.ply", *icp_object);
	*/

	pcl::console::print_highlight("ICP...\n");
	icp.setInputSource(icp_pose);
	icp.setInputTarget(icp_object);
	icp.setMaxCorrespondenceDistance(100);
	icp.setTransformationEpsilon(1e-10);
	icp.setEuclideanFitnessEpsilon(1);
	icp.setMaximumIterations(100);
	//icp.align(*icp_object);

	if (icp.hasConverged())
	{
		// Print results
		printf("\n");
		Eigen::Matrix4f transformation = icp.getFinalTransformation();
		transformation = transformation.inverse().eval();
		pcl::transformPointCloud(*icp_object, *icp_object, transformation);
		pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1), transformation(0, 2));
		pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1), transformation(1, 2));
		pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1), transformation(2, 2));
		pcl::console::print_info("\n");
		pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3), transformation(2, 3));
		pcl::console::print_info("\n");

		pcl::visualization::PCLVisualizer visu("Alignment");
		visu.addPointCloud(icp_scene, ColorHandlerT(icp_scene, 0.0, 255.0, 0.0), "scene");
		visu.addPointCloud(icp_object, ColorHandlerT(icp_object, 0.0, 0.0, 255.0), "object_aligned");
		visu.addPointCloud(icp_pose, ColorHandlerT(icp_pose, 0.0, 255.0, 255.0), "object_move");
		visu.spin();
	}


	
	/*
	// Perform alignment
	pcl::console::print_highlight("Starting alignment...\n");
	pcl::SampleConsensusPrerejective<pcl::XYZRGBNormal, pcl::XYZRGBNormal, pcl::GASDSignature984> align;
	align.setInputSource(object);
	align.setSourceFeatures(object_features);
	align.setInputTarget(scene);
	align.setTargetFeatures(scene_features);
	align.setMaximumIterations(50000); // Number of RANSAC iterations
	align.setNumberOfSamples(3); // Number of points to sample for generating/prerejecting a pose
	align.setCorrespondenceRandomness(5); // Number of nearest features to use
	align.setSimilarityThreshold(0.9f); // Polygonal edge length similarity threshold
	align.setMaxCorrespondenceDistance(2.5f * leaf); // Inlier threshold
	align.setInlierFraction(0.25f); // Required inlier fraction for accepting a pose hypothesis
	{
		pcl::ScopeTime t("Alignment");
		align.align(*object_aligned);
	}

	if (align.hasConverged())
	{
		// Print results
		printf("\n");
		Eigen::Matrix4f transformation = align.getFinalTransformation();
		pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1), transformation(0, 2));
		pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1), transformation(1, 2));
		pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1), transformation(2, 2));
		pcl::console::print_info("\n");
		pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3), transformation(2, 3));
		pcl::console::print_info("\n");
		pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(), object->size());

		// Show alignment
		pcl::visualization::PCLVisualizer visu("Alignment");
		visu.addPointCloud(scene, ColorHandlerT(scene, 0.0, 255.0, 0.0), "scene");
		visu.addPointCloud(object_aligned, ColorHandlerT(object_aligned, 0.0, 0.0, 255.0), "object_aligned");
		visu.spin();
	}
	else
	{
		pcl::console::print_error("Alignment failed!\n");
		return (1);
	}
	*/
	system("pause");
	return (0);
}