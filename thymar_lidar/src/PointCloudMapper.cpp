#include "PointCloudMapper.hpp"

PointCloudMapper::PointCloudMapper(int grid_width, int grid_height, float grid_resolution){
	this->grid_width = grid_width;
	this->grid_height = grid_height;
	this->grid_resolution = grid_resolution;

	this->occupancy_grid.resize(grid_width*grid_height);
	std::fill(this->occupancy_grid.begin(),this->occupancy_grid.end(),-1);

	
}

void PointCloudMapper::addPointCloud(pcl::PointCloud<pcl::PointXYZ> new_point_cloud, Pose2d pose2d){
	if(this->first){
		this->world_point_cloud=new_point_cloud;
		this->first=false;
	}else{
		this->world_point_cloud+=this->transform(new_point_cloud, pose2d.x,pose2d.y,0.0,pose2d.theta);;
		std::cout << pose2d.x << pose2d.y << pose2d.theta << std::endl;
	}

	this->world_point_cloud = this->downSample(this->world_point_cloud.makeShared());

	float x,y,z;
    for(int n=0; n<this->world_point_cloud.points.size(); n++){
    	x = this->world_point_cloud.points[n].x;
    	y = this->world_point_cloud.points[n].y;
    	z =this->world_point_cloud.points[n].z;

    	int xi = (int) std::round(x/grid_resolution);
    	int yi = (int) std::round(y/grid_resolution);
    	if(z > 0.10){
    		this->occupancy_grid[(yi+(this->grid_height/2))*this->grid_width+xi+(this->grid_width/2)] = 100;

    	}else{
    		;//grid_data[(yi+60)*120+xi+60] = 0;
    	}
    }
}

pcl::PointCloud<pcl::PointXYZ> PointCloudMapper::downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud){
	pcl::PointCloud<pcl::PointXYZ> target_cloud;

	pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(source_cloud);
    sor.setLeafSize(0.1,0.1,0.1);
    sor.filter(target_cloud);
    return target_cloud;  
}

std::vector<signed char> PointCloudMapper::getOccupancyGrid(){
	return this->occupancy_grid;
}

pcl::PointCloud<pcl::PointXYZ> PointCloudMapper::getWorldPointCloud(){
	return this->world_point_cloud;
}

pcl::PointCloud<pcl::PointXYZ> PointCloudMapper::transform(pcl::PointCloud<pcl::PointXYZ> source_cloud, float x, float y, float z, float theta){
	Eigen::Affine3f transform = Eigen::Affine3f::Identity();
	transform.translation() << x,y,z;
	transform.rotate(Eigen::AngleAxisf(theta,Eigen::Vector3f::UnitZ()));

	pcl::PointCloud<pcl::PointXYZ> target_cloud;

	pcl::transformPointCloud(source_cloud, target_cloud, transform);
	return target_cloud;
}


