#include <pcl/filters/voxel_grid.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/common/time.h>
#include <iostream>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/registration/sample_consensus_prerejective.h>

void pose_estimation_of_rigid_objects(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
	// Point clouds
	pcl::PointCloud<pcl::PointNormal>::Ptr object(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr object_aligned(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr scene(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features(new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features(new pcl::PointCloud<pcl::FPFHSignature33>);

	pcl::copyPointCloud(*cloud, *scene);

	// Load object and scene
	pcl::console::print_highlight("Loading point clouds...\n");
	if (pcl::io::loadPLYFile<pcl::PointNormal>("./model/sofa5000.ply", *object) < 0)
	{
		pcl::console::print_error("Error loading object/scene file!\n");
		return;
	}

	// Downsample
	pcl::console::print_highlight("Downsampling...\n");
	pcl::VoxelGrid<pcl::PointNormal> grid;
	const float leaf = 0.025f;
	grid.setLeafSize(leaf, leaf, leaf);
	//grid.setInputCloud(object);
	//grid.filter(*object);
	grid.setInputCloud(scene);
	grid.filter(*scene);
	cout << "Scene points:" << scene->size() << endl;

	Eigen::Matrix4f scale = Eigen::Matrix4f::Identity();
	scale(0, 0) = 0.915378;
	scale(1, 1) = 0.703405;
	scale(2, 2) = 0.9440087;
	pcl::transformPointCloud(*object, *object, scale);

	/*
	Eigen::Matrix4f translate = Eigen::Matrix4f::Identity();

	pcl::compute3DCentroid(*scene, centroid);
	translate(0, 3) = centroid[0];
	translate(1, 3) = centroid[1];
	translate(2, 3) = centroid[2];
	pcl::compute3DCentroid(*object, centroid);
	translate(0, 3) -= centroid[0];
	translate(1, 3) -= centroid[1];
	translate(2, 3) -= centroid[2];

	pcl::transformPointCloud(*object, *object, translate);
	*/
	// Estimate normals for scene
	pcl::console::print_highlight("Estimating scene normals...\n");
	pcl::NormalEstimationOMP<pcl::PointNormal, pcl::PointNormal> nest;
	nest.setNumberOfThreads(8);
	nest.setRadiusSearch(0.025);
	nest.setInputCloud(scene);
	nest.compute(*scene);
	

	// Estimate normals for object
	pcl::console::print_highlight("Estimating object normals...\n");
	nest.setInputCloud(object);
	nest.compute(*object);

	// Estimate features
	pcl::console::print_highlight("Estimating features...\n");
	pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fest;
	fest.setNumberOfThreads(8);
	fest.setRadiusSearch(0.025);
	fest.setInputCloud(object);
	fest.setInputNormals(object);
	fest.compute(*object_features);
	fest.setInputCloud(scene);
	fest.setInputNormals(scene);
	fest.compute(*scene_features);


	// Perform alignment
	pcl::console::print_highlight("Starting alignment...\n");
	pcl::SampleConsensusPrerejective<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> align;
	align.setInputSource(scene);
	align.setSourceFeatures(scene_features);
	align.setInputTarget(object);
	align.setTargetFeatures(object_features);
	align.setMaximumIterations(40000); // Number of RANSAC iterations
	align.setNumberOfSamples(3); // Number of points to sample for generating/prerejecting a pose
	align.setCorrespondenceRandomness(5); // Number of nearest features to use
	align.setSimilarityThreshold(0.92f); // Polygonal edge length similarity threshold
	align.setMaxCorrespondenceDistance(0.01); // Inlier threshold
	align.setTransformationEpsilon(1e-5);

	{
		pcl::ScopeTime t("Alignment");
		align.align(*object_aligned);
	}

	if (align.hasConverged())
	{
		// Print results
		printf("\n");
		Eigen::Matrix4f transformation = align.getFinalTransformation().inverse();
		pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(0, 0), transformation(0, 1), transformation(0, 2));
		pcl::console::print_info("R = | %6.3f %6.3f %6.3f | \n", transformation(1, 0), transformation(1, 1), transformation(1, 2));
		pcl::console::print_info("    | %6.3f %6.3f %6.3f | \n", transformation(2, 0), transformation(2, 1), transformation(2, 2));
		pcl::console::print_info("\n");
		pcl::console::print_info("t = < %0.3f, %0.3f, %0.3f >\n", transformation(0, 3), transformation(1, 3), transformation(2, 3));
		pcl::console::print_info("\n");
		pcl::console::print_info("Inliers: %i/%i\n", align.getInliers().size(), scene->size());
		cout << "Score:" << align.getFitnessScore() << endl;
		// Show alignment    
		pcl::copyPointCloud(*object, *object_aligned);
		pcl::transformPointCloud(*object_aligned, *object_aligned, transformation);
		viewer.addPointCloud(scene, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(scene, 0.0, 255.0, 0.0), "scene");
		viewer.addPointCloud(object_aligned, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(object_aligned, 0.0, 0.0, 255.0), "object_aligned");
		viewer.addPointCloud(object, pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal>(object, 255.0, 0.0, 0.0), "object");
	}
	else
	{
		pcl::console::print_error("Alignment failed!\n");
		return;
	}
}

void SAC()
{
	//SAC-IA
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tc_handler(sample_cloud, 255, 0, 0);
	viewer.addPointCloud(sample_cloud, tc_handler, "model_cloud");

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tc_handler1(e_cloud, 0, 255, 0);
	viewer.addPointCloud(e_cloud, tc_handler1, "e_cloud");

	pcl::PointCloud<pcl::FPFHSignature33>::Ptr object_features(new pcl::PointCloud<pcl::FPFHSignature33>);
	pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features(new pcl::PointCloud<pcl::FPFHSignature33>);

	pcl::FPFHEstimationOMP<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> fest;
	fest.setNumberOfThreads(8);
	fest.setRadiusSearch(0.05);
	fest.setInputCloud(sample_cloud_normal);
	fest.setInputNormals(sample_cloud_normal);
	fest.compute(*object_features);
	fest.setInputCloud(cloud_cluster);
	fest.setInputNormals(cloud_cluster);
	fest.compute(*scene_features);
	cout << "Feature done!!" << endl;

	t = clock();

	pcl::SampleConsensusInitialAlignment<pcl::PointNormal, pcl::PointNormal, pcl::FPFHSignature33> scia;
	scia.setInputSource(cloud_cluster);
	scia.setInputTarget(sample_cloud_normal);
	scia.setSourceFeatures(scene_features);
	scia.setTargetFeatures(object_features);
	scia.setMinSampleDistance(0.2);
	//scia.setMaxCorrespondenceDistance(0.01);
	//scia.setMaximumIterations(100);
	scia.setNumberOfSamples(3);
	scia.setCorrespondenceRandomness(20);
	scia.setEuclideanFitnessEpsilon(0.0001);
	//scia.setTransformationEpsilon(1e-10);
	//scia.setRANSACIterations(30);
	pcl::PointCloud<pcl::PointNormal>::Ptr test_cloud(new pcl::PointCloud<pcl::PointNormal>);
	scia.align(*test_cloud);
	cout << "sac has converged:" << scia.hasConverged() << "  score: " << scia.getFitnessScore() << endl;
	cout << "Sac Time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> single_color(test_cloud, 70, 255 / (1 + 1), 70);
	viewer.addPointCloud<pcl::PointNormal>(test_cloud, single_color, "test");
}

/* ------------------------------SHOT descriptor--------------------
//-----------------Uniform Sampling--------------

pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
uniform_sampling.setInputCloud(sample_cloud);
uniform_sampling.setRadiusSearch(0.1f);
uniform_sampling.filter(*model_keypoints);
cout << "Model total points: " << sample_cloud->size() << "; Selected Keypoints: " << model_keypoints->size() << endl;
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sample_cloud_handler(model_keypoints, 255, 0, 0);
viewer.addPointCloud(model_keypoints, sample_cloud_handler, "model_keypoints");
viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "model_keypoints");

uniform_sampling.setInputCloud(e_cloud);
uniform_sampling.setRadiusSearch(0.1f);
uniform_sampling.filter(*scene_keypoints);
cout << "Scene total points: " << e_cloud->size() << "; Selected Keypoints: " << scene_keypoints->size() << endl;
pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> e_cloud_handler(scene_keypoints, 255, 0, 0);
viewer.addPointCloud(scene_keypoints, e_cloud_handler, "scene_keypoints");
viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

//show_normals(viewer, cloud_cluster, "1");
//show_normals(viewer, sample_cloud_normal, "2");

//  Compute Descriptor for keypoints
t = clock();
cout << "Compute Descriptor Strart!!" << endl;
pcl::SHOTEstimationOMP<pcl::PointXYZ, pcl::Normal, pcl::SHOT352> descr_est;
descr_est.setRadiusSearch(0.25f);
descr_est.setNumberOfThreads(8);

descr_est.setInputCloud(model_keypoints);
descr_est.setInputNormals(sample_normal);
descr_est.setSearchSurface(sample_cloud);
descr_est.compute(*model_descriptors);

descr_est.setInputCloud(scene_keypoints);
descr_est.setInputNormals(e_normal);
descr_est.setSearchSurface(e_cloud);
descr_est.compute(*scene_descriptors);

cout << "Done!!" << " Time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;

//  Find Model-Scene Correspondences with KdTree
pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());


t = clock();
cout << "Matching Descriptor Strart!!" << endl;
pcl::KdTreeFLANN<pcl::SHOT352> match_search;
match_search.setInputCloud(model_descriptors);

//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
for (size_t i = 0; i < scene_descriptors->size(); ++i)
{
std::vector<int> neigh_indices(3);
std::vector<float> neigh_sqr_dists(3);
if (!pcl_isfinite(scene_descriptors->at(i).descriptor[0])) //skipping NaNs
{
continue;
}
int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);

cout << neigh_sqr_dists[0] << endl;

if (found_neighs > 0) //  add match only if the squared descriptor distance is less than 0.5 (SHOT descriptor distances are between 0 and 2 by design)
{
for (int j = 0; j < neigh_indices.size(); j++)
{
pcl::Correspondence corr(neigh_indices[j], static_cast<int> (i), neigh_sqr_dists[j]);
model_scene_corrs->push_back(corr);
}

}
}
cout << "Done!!" << " Time : " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;
std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;

for (int i = 0; i < model_scene_corrs->size(); i++)
{

viewer.addLine(scene_keypoints->points[model_scene_corrs->at(i).index_match], model_keypoints->points[model_scene_corrs->at(i).index_query], 0, 255, 0, "C"+to_string(i));

}
---------------------------------SHOT descriptor------------------------------*/

Eigen::Matrix4f object_coordinate(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud, string id)
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr normal(new pcl::PointCloud<pcl::PointXYZ>);

	for (int i = 0; i < cloud->size(); i++)
	{
		pcl::PointXYZ point;

		point.x = cloud->points[i].normal_x;
		point.y = cloud->points[i].normal_y;
		point.z = cloud->points[i].normal_z;

		normal->points.push_back(point);
	}
	//viewer.addPointCloud(normal, "cloud10"+id);

	Eigen::Vector4f pcaCentroid;
	pcl::compute3DCentroid(*cloud, pcaCentroid);
	Eigen::Matrix3f covariance;
	//pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);	
	pcl::computeCovarianceMatrixNormalized(*normal, Eigen::Vector4f(0, 0, 0, 0), covariance);
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
	Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
	Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();

	//get Z-axis
	Eigen::Vector3f prime = eigenVectorsPCA.col(2);
	int positive = 0, negitive = 0;
	float dot_product = 0.0;
	for (int i = 0; i < normal->size(); i++)
	{
		Eigen::Vector3f n = normal->points[i].getVector3fMap();
		dot_product += prime.dot(n);
	}

	cout << "DOT:" << dot_product << endl;

	if (dot_product < 0)
	{
		eigenVectorsPCA.col(2) *= -1;
	}

	//get X-axis or Y-axis depend on eigenValues
	if (eigenValuesPCA(0) > eigenValuesPCA(1))
	{
		cout << "X-axis" << endl;
		prime = eigenVectorsPCA.col(0);
		positive = 0, negitive = 0; dot_product = 0.0;
		for (int i = 0; i < normal->size(); i++)
		{
			Eigen::Vector3f n = normal->points[i].getVector3fMap();
			dot_product += prime.dot(n);
		}
		cout << "DOT:" << dot_product << endl;

		if (dot_product < 0)
		{
			eigenVectorsPCA.col(0) *= -1;
		}
		eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0)); // get Y-axis by cross Z-axis and X-axis
	}
	else
	{
		cout << "Y-axis" << endl;
		prime = eigenVectorsPCA.col(1);
		positive = 0, negitive = 0; dot_product = 0.0;
		for (int i = 0; i < normal->size(); i++)
		{
			Eigen::Vector3f n = normal->points[i].getVector3fMap();
			dot_product += prime.dot(n);
		}
		cout << "DOT:" << dot_product << endl;

		if (dot_product < 0)
		{
			eigenVectorsPCA.col(1) *= -1;
		}
		eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2)); // get X-axis by cross Z-axis and Y-axis
	}


	Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
	Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
	tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.	
	tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t

																						  //pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
																						  //pcl::transformPointCloudWithNormals(*cloud, *transformedCloud, tm);

	std::cout << eigenValuesPCA << std::endl;
	std::cout << eigenVectorsPCA << std::endl;

	/*
	pcl::PointXYZ c;
	c.x = pcaCentroid(0);
	c.y = pcaCentroid(1);
	c.z = pcaCentroid(2);
	pcl::PointXYZ pcZ;
	pcZ.x = eigenVectorsPCA(0, 2) + c.x;
	pcZ.y = eigenVectorsPCA(1, 2) + c.y;
	pcZ.z = eigenVectorsPCA(2, 2) + c.z;
	pcl::PointXYZ pcY;
	pcY.x = eigenVectorsPCA(0, 1) + c.x;
	pcY.y = eigenVectorsPCA(1, 1) + c.y;
	pcY.z = eigenVectorsPCA(2, 1) + c.z;
	pcl::PointXYZ pcX;
	pcX.x = eigenVectorsPCA(0, 0) + c.x;
	pcX.y = eigenVectorsPCA(1, 0) + c.y;
	pcX.z = eigenVectorsPCA(2, 0) + c.z;

	viewer.addArrow(pcZ, c, 0.0, 0.0, 1.0, false, "arrow_z"+id);
	viewer.addArrow(pcY, c, 0.0, 1.0, 0.0, false, "arrow_y"+id);
	viewer.addArrow(pcX, c, 1.0, 0.0, 0.0, false, "arrow_x"+id);

	pcl::PointXYZ o(0, 0, 0);
	pcl::PointXYZ Z;
	Z.x = eigenVectorsPCA(0, 2);
	Z.y = eigenVectorsPCA(1, 2);
	Z.z = eigenVectorsPCA(2, 2);
	pcl::PointXYZ Y;
	Y.x = eigenVectorsPCA(0, 1);
	Y.y = eigenVectorsPCA(1, 1);
	Y.z = eigenVectorsPCA(2, 1);
	pcl::PointXYZ X;
	X.x = eigenVectorsPCA(0, 0);
	X.y = eigenVectorsPCA(1, 0);
	X.z = eigenVectorsPCA(2, 0);


	viewer.addArrow(Z, o, 0.0, 0.0, 1.0, false, "arrow_z1" +id );
	viewer.addArrow(Y, o, 0.0, 1.0, 0.0, false, "arrow_y1" +id );
	viewer.addArrow(X, o, 1.0, 0.0, 0.0, false, "arrow_x1" +id );

	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> single_color(cloud, 70, 255 / (1 + 1), 70);
	viewer.addPointCloud<pcl::PointXYZRGBNormal>(cloud, single_color, "cloud" + id);
	*/
	//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGBNormal> tc_handler(transformedCloud, 0, 255/stoi(id), 0); 	
	//viewer.addPointCloud<pcl::PointXYZRGBNormal>(transformedCloud, tc_handler, "transformCloud"+id);


	return tm;
}

void plane_fitting(pcl::visualization::PCLVisualizer& viewer, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr model, vector<Eigen::Vector3f> normals_c, vector<pcl::PointXYZ> box_centers_c, vector<Eigen::Vector3f> normals_m, vector<pcl::PointXYZ> box_centers_m)
{
	clock_t t;
	pcl::PointCloud<pcl::PointXYZ>::Ptr temp(new pcl::PointCloud<pcl::PointXYZ>), temp2(new pcl::PointCloud<pcl::PointXYZ>), final_cloud(new pcl::PointCloud<pcl::PointXYZ>);

	//pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
	//kdtree.setInputCloud(cloud);

	std::vector<int> pointIdxNKNSearch(1);
	std::vector<float> pointNKNSquaredDistance(1);
	float Min_E = -1;
	t = clock();
	cout << "Plane Fitting start!!" << endl;
	for (int i = 0; i < normals_c.size(); i++)
	{
		for (int j = 0; j < normals_m.size(); j++)
		{
			Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();

			transformation(0, 3) = -box_centers_m[j].x;
			transformation(1, 3) = -box_centers_m[j].y;
			transformation(2, 3) = -box_centers_m[j].z;
			pcl::transformPointCloud(*model, *temp, transformation);

			transformation = process.CreateRotateMatrix(normals_m[j], normals_c[i]);

			for (int k = 0; k < 19; k++)
			{

				Eigen::Matrix4f T = Eigen::Matrix4f::Identity();

				Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
				transform_2.rotate(Eigen::AngleAxisf(M_PI / 18 * k, normals_c[i]));;

				T = transform_2.matrix() * transformation;

				T(0, 3) = box_centers_c[0].x;
				T(1, 3) = box_centers_c[0].y;
				T(2, 3) = box_centers_c[0].z;

				pcl::transformPointCloud(*temp, *temp2, T);

				pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
				kdtree.setInputCloud(temp2);

				//two point cloud's Euclidean
				float E = 0.0;
				for (int p = 0; p < cloud->size(); p++)
				{
					if (kdtree.nearestKSearch(cloud->points[p], 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
					{
						E = E + pointNKNSquaredDistance[0];
					}
				}

				if (Min_E == -1 || Min_E > E)
				{
					Min_E = E;
					pcl::copyPointCloud(*temp2, *final_cloud); //copy best model cloud
				}
				cout << E << endl;
			}
			/*
			//rotate 180
			transformation = Eigen::Matrix4f::Identity();

			transformation(0, 3) = -box_centers_m[j].x;
			transformation(1, 3) = -box_centers_m[j].y;
			transformation(2, 3) = -box_centers_m[j].z;
			pcl::transformPointCloud(*model, *temp, transformation);

			transformation = process.CreateRotateMatrix(normals_m[j]*-1, normals_c[i]);
			pcl::transformPointCloud(*temp, *temp, transformation);

			transformation = Eigen::Matrix4f::Identity();
			transformation(0, 3) = box_centers_c[i].x;
			transformation(1, 3) = box_centers_c[i].y;
			transformation(2, 3) = box_centers_c[i].z;
			pcl::transformPointCloud(*temp, *temp, transformation);


			pcl::visualization::PointCloudColorHandlerRandom<pcl::PointXYZ>tc_handler1(temp);
			//pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tc_handler(cloud_f, 0, 255/number, 0);
			viewer.addPointCloud(temp, tc_handler1, "align" + to_string(i) + to_string(j)+ "0");
			*/
		}
	}
	cout << "Plane Fitting time: " << float((clock() - t)) / CLOCKS_PER_SEC << " s" << endl;
	pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> tc_handler(final_cloud, 255, 0, 255);
	viewer.addPointCloud(final_cloud, tc_handler, "align");
}

void point_transform(pcl::PointXYZ &p, pcl::PointXYZ center, Eigen::Matrix4f trans)
{
	Eigen::Vector4f temp;

	temp[0] = p.x;
	temp[1] = p.y;
	temp[2] = p.z;
	temp[3] = 0;

	temp = trans * temp;

	p.x = temp[0] + center.x;
	p.y = temp[1] + center.y;
	p.z = temp[2] + center.z;


}

for (int i = 0; i < process.get_walls_number(); i++)
{
	Eigen::Vector3f normal;
	pcl::PointXYZ minpt, maxpt;
	Eigen::Matrix4f temp_transformation;

	normal[0] = process.get_walls_coefficients(i).x;
	normal[1] = 0;
	normal[2] = process.get_walls_coefficients(i).z;

	if (i == 0)
		temp_transformation = Eigen::Matrix4f::Identity();
	else
		temp_transformation = process.CreateRotateMatrix(Eigen::Vector3f::UnitZ(), normal);

	minpt.x = -process.get_walls_size(i).x / 2;
	minpt.y = -process.get_walls_size(i).y / 2;
	minpt.z = -process.get_walls_size(i).z / 2;

	maxpt.x = process.get_walls_size(i).x / 2;
	maxpt.y = process.get_walls_size(i).y / 2;
	maxpt.z = process.get_walls_size(i).z / 2;


	pcl::PointXYZ pt1(minpt.x, minpt.y, minpt.z);
	point_transform(pt1, process.get_walls_center(i), temp_transformation);
	pcl::PointXYZ pt2(minpt.x, minpt.y, maxpt.z);
	point_transform(pt2, process.get_walls_center(i), temp_transformation);
	pcl::PointXYZ pt3(maxpt.x, minpt.y, maxpt.z);
	point_transform(pt3, process.get_walls_center(i), temp_transformation);
	pcl::PointXYZ pt4(maxpt.x, minpt.y, minpt.z);
	point_transform(pt4, process.get_walls_center(i), temp_transformation);
	pcl::PointXYZ pt5(minpt.x, maxpt.y, minpt.z);
	point_transform(pt5, process.get_walls_center(i), temp_transformation);
	pcl::PointXYZ pt6(minpt.x, maxpt.y, maxpt.z);
	point_transform(pt6, process.get_walls_center(i), temp_transformation);
	pcl::PointXYZ pt7(maxpt.x, maxpt.y, maxpt.z);
	point_transform(pt7, process.get_walls_center(i), temp_transformation);
	pcl::PointXYZ pt8(maxpt.x, maxpt.y, minpt.z);
	point_transform(pt8, process.get_walls_center(i), temp_transformation);

	viewer.addLine(pt1, pt2, 1.0, 0.0, 0.0, "1 edge" + to_string(i));
	viewer.addLine(pt1, pt4, 1.0, 0.0, 0.0, "2 edge" + to_string(i));
	viewer.addLine(pt1, pt5, 1.0, 0.0, 0.0, "3 edge" + to_string(i));
	viewer.addLine(pt5, pt6, 1.0, 0.0, 0.0, "4 edge" + to_string(i));
	viewer.addLine(pt5, pt8, 1.0, 0.0, 0.0, "5 edge" + to_string(i));
	viewer.addLine(pt2, pt6, 1.0, 0.0, 0.0, "6 edge" + to_string(i));
	viewer.addLine(pt6, pt7, 1.0, 0.0, 0.0, "7 edge" + to_string(i));
	viewer.addLine(pt7, pt8, 1.0, 0.0, 0.0, "8 edge" + to_string(i));
	viewer.addLine(pt2, pt3, 1.0, 0.0, 0.0, "9 edge" + to_string(i));
	viewer.addLine(pt4, pt8, 1.0, 0.0, 0.0, "10 edge" + to_string(i));
	viewer.addLine(pt3, pt4, 1.0, 0.0, 0.0, "11 edge" + to_string(i));
	viewer.addLine(pt3, pt7, 1.0, 0.0, 0.0, "12 edge" + to_string(i));

	//viewer.addSphere(minpt, 0.1, "sphere min" + to_string(i));
	//viewer.addSphere(maxpt, 0.1, "sphere max" + to_string(i));
}