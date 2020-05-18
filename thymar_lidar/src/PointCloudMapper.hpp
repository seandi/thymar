#include <iostream>
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/extract_clusters.h>


struct Pose2d{
	double x,y,theta;
};

struct Velocity{
	double linear, angular;
};

struct SphereModel{
	float x,y,radius;
};


class PointCloudMapper{
private:
	pcl::PointCloud<pcl::PointXYZ> world_point_cloud;
	pcl::PointCloud<pcl::PointXYZ> terrain_point_cloud;
	pcl::PointCloud<pcl::PointXYZ> obstacles_point_cloud;
	pcl::PointCloud<pcl::PointXYZ> model_cloud;

	std::vector<signed char> occupancy_grid;
	SphereModel target_model;
	bool target_found = false;
	int grid_width;
	int grid_height;
	float grid_resolution;
	bool first = true;

	pcl::PointCloud<pcl::PointXYZ> downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud);
	pcl::PointCloud<pcl::PointXYZ> transform(pcl::PointCloud<pcl::PointXYZ> source_cloud, float x, float y, float z, float theta);
	pcl::PointCloud<pcl::PointXYZ> directionalFilter(
		pcl::PointCloud<pcl::PointXYZ> source_cloud, std::string axis, float lower_bound, float upper_bound,
		pcl::PointCloud<pcl::PointXYZ>& outliers_cloud
	);
public:
	PointCloudMapper(int grid_width=10, int grid_height=10, float grid_resolution=0.05);
	void addPointCloud(pcl::PointCloud<pcl::PointXYZ> new_point_cloud, Pose2d pose2d);
	std::vector<float> fitSphere(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr inliers_cloud=NULL);
	std::vector<float> locateSphere(pcl::PointCloud<pcl::PointXYZ> input_cloud);

	std::vector<signed char> getOccupancyGrid();
	pcl::PointCloud<pcl::PointXYZ> getWorldPointCloud();
	pcl::PointCloud<pcl::PointXYZ> getObstaclesPointCloud(){return this->obstacles_point_cloud; };
	pcl::PointCloud<pcl::PointXYZ> getTerrainPointCloud(){return this->terrain_point_cloud;};
	SphereModel getTargetModel(){return this->target_model;};
	bool isTargetFound(){return this->target_found;};



};