#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

using namespace std;

class FeatureCloud
{
	public:
		typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
		typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
		typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
		typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

		FeatureCloud() :
			search_method_xyz_(new SearchMethod),
			normal_radius_(0.02f),
			feature_radius_(0.02f)
		{}

		~FeatureCloud() {}

		void setInputCloud(PointCloud::Ptr xyz)
		{
			xyz_ = xyz;
			processInput();
		}

		void loadInputCloud(const std::string &pcd_file)
		{
			xyz_ = PointCloud::Ptr(new PointCloud);
			pcl::io::loadPLYFile(pcd_file, *xyz_);
			processInput();
		}

		PointCloud::Ptr getPointCloud() const
		{
			return (xyz_);
		}

		SurfaceNormals::Ptr getSurfaceNormals() const
		{
			return (normals_);
		}

		LocalFeatures::Ptr getLocalFeatures() const
		{
			return (features_);
		}

	protected:
		void processInput()
		{
			computeSurfaceNormals();
			computeLocalFeatures();
		}

		void computeSurfaceNormals()
		{
			normals_ = SurfaceNormals::Ptr(new SurfaceNormals);
			pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
			norm_est.setInputCloud(xyz_);
			norm_est.setSearchMethod(search_method_xyz_);
			norm_est.setRadiusSearch(normal_radius_);
			norm_est.compute(*normals_);
		}

		void computeLocalFeatures()
		{
			features_ = LocalFeatures::Ptr(new LocalFeatures);
			pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
			fpfh_est.setInputCloud(xyz_);
			fpfh_est.setInputNormals(normals_);
			fpfh_est.setSearchMethod(search_method_xyz_);
			fpfh_est.setRadiusSearch(feature_radius_);
			fpfh_est.compute(*features_);
		}
	private:
		PointCloud::Ptr xyz_;
		SurfaceNormals::Ptr normals_;
		LocalFeatures::Ptr features_;
		SearchMethod::Ptr search_method_xyz_;

		float normal_radius_;
		float feature_radius_;
};

class TemplateAlignment
{
	public:
		struct Result
		{
			float fitness_score;
			Eigen::Matrix4f final_transformation;
			EIGEN_MAKE_ALIGNED_OPERATOR_NEW
		};
		TemplateAlignment() :
			min_sample_distance_(0.05f),
			max_correspondence_distance_(0.01f*0.01f),
			nr_iterations_(500)
		{
			sac_ia_.setMinSampleDistance(min_sample_distance_);
			sac_ia_.setMaxCorrespondenceDistance(max_correspondence_distance_);
			sac_ia_.setMaximumIterations(nr_iterations_);
		}

		~TemplateAlignment() {}

		void setTargetCloud(FeatureCloud &target_cloud)
		{
			target_ = target_cloud;
			sac_ia_.setInputTarget(target_cloud.getPointCloud());
			sac_ia_.setTargetFeatures(target_cloud.getLocalFeatures());
		}

		void addTemplateCloud(FeatureCloud &template_cloud)
		{
			templates_.push_back(template_cloud);
		}

		void align(FeatureCloud &template_cloud, TemplateAlignment::Result &result)
		{
			sac_ia_.setInputCloud(template_cloud.getPointCloud());
			sac_ia_.setSourceFeatures(template_cloud.getLocalFeatures());

			pcl::PointCloud<pcl::PointXYZ> registration_output;
			sac_ia_.align(registration_output);

			result.fitness_score = (float)sac_ia_.getFitnessScore(max_correspondence_distance_);
			result.final_transformation = sac_ia_.getFinalTransformation();
		}
		
	private:
		std::vector<FeatureCloud> templates_;
		FeatureCloud target_;

		pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
		float min_sample_distance_;
		float max_correspondence_distance_;
		int nr_iterations_;
};


int main(int argc, char **argv)
{
	pcl::PLYReader Reader;
	pcl::PLYWriter Writer;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr template_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	FeatureCloud template_cloud;
	template_cloud.loadInputCloud(argv[1]);
	Reader.read(argv[2], *target_cloud);

	pcl::console::print_highlight("removing distant points...\n");
	const float depth_limit = 1.0;
	pcl::PassThrough<pcl::PointXYZ> pass;
	pass.setInputCloud(target_cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(0, depth_limit);
	pass.filter(*target_cloud);

	pcl::console::print_highlight("Downsampling...\n");
	const float voxel_grid_size = 0.005f;
	pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
	vox_grid.setInputCloud(target_cloud);
	vox_grid.setLeafSize(voxel_grid_size, voxel_grid_size, voxel_grid_size);
	pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud(new pcl::PointCloud<pcl::PointXYZ>);
	vox_grid.filter(*tempCloud);
	target_cloud = tempCloud;

	FeatureCloud cloud;
	cloud.setInputCloud(target_cloud);

	TemplateAlignment template_align;
	template_align.addTemplateCloud(template_cloud);
	template_align.setTargetCloud(cloud);
	TemplateAlignment::Result best_alignment;
	const FeatureCloud &best_template = template_cloud;

	Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3, 3>(0, 0);
	Eigen::Vector3f translation = best_alignment.final_transformation.block<3, 1>(0, 3);

	printf("\n");
	printf("    | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
	printf("R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
	printf("    | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
	printf("\n");
	printf("t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));

	pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
	pcl::transformPointCloud(*best_template.getPointCloud(), transformed_cloud, best_alignment.final_transformation);
	Writer.write("output.ply", transformed_cloud);

	system("pause");
	return (0);
}



