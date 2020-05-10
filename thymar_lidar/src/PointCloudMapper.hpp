#include <iostream>
#include "pcl/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/voxel_grid.h>


class PointCloudMapper{
private:
	pcl::PointCloud<pcl::PointXYZ> world_point_cloud;
	std::vector<signed char> occupancy_grid;
	int grid_width;
	int grid_height;
	float grid_resolution;

	pcl::PointCloud<pcl::PointXYZ> downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud);
public:
	PointCloudMapper(int grid_width=10, int grid_height=10, float grid_resolution=0.05);
	void addPointCloud(pcl::PointCloud<pcl::PointXYZ> new_point_cloud);
	std::vector<signed char> getOccupancyGrid();
	pcl::PointCloud<pcl::PointXYZ> getWorldPointCloud();


};