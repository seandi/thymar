#include <iostream>
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>



struct Pose2d{
	double x,y,theta;
};


class PointCloudMapper{
private:
	pcl::PointCloud<pcl::PointXYZ> world_point_cloud;
	pcl::PointCloud<pcl::PointXYZ> terrain_point_cloud;
	pcl::PointCloud<pcl::PointXYZ> obstacles_point_cloud;
	std::vector<signed char> occupancy_grid;
	int grid_width;
	int grid_height;
	float grid_resolution;
	bool first = true;

	pcl::PointCloud<pcl::PointXYZ> downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud);
	pcl::PointCloud<pcl::PointXYZ> transform(pcl::PointCloud<pcl::PointXYZ> source_cloud, float x, float y, float z, float theta);
	pcl::PointCloud<pcl::PointXYZ> directionalFilter(
		pcl::PointCloud<pcl::PointXYZ> source_cloud, std::string axis, float lower_bound, float upper_bound,
		pcl::PointCloud<pcl::PointXYZ>::Ptr outliers_cloud=NULL
	);
public:
	PointCloudMapper(int grid_width=10, int grid_height=10, float grid_resolution=0.05);
	void addPointCloud(pcl::PointCloud<pcl::PointXYZ> new_point_cloud, Pose2d pose2d);
	std::vector<signed char> getOccupancyGrid();
	pcl::PointCloud<pcl::PointXYZ> getWorldPointCloud();


};