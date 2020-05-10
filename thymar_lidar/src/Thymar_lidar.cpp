#include "ros/ros.h"
#include <iostream>
#include "pcl/point_cloud.h"
#include <pcl_ros/point_cloud.h>
#include "pcl_conversions/pcl_conversions.h"
#include "nav_msgs/OccupancyGrid.h"
#include <string>
#include "PointCloudMapper.hpp"


class ThymarLidar{
private:
	ros::NodeHandle nh;
	ros::Rate rate = ros::Rate(10.0);
    ros::Subscriber pointcloud_subscriber;
    ros::Publisher pointcloud_publisher;
    ros::Publisher grid_publisher;


    std::string name;
    PointCloudMapper* mapper;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    nav_msgs::OccupancyGrid grid;
    bool new_point_cloud = false;

public:
	ThymarLidar(int argc, char** argv, float hz=10, int grid_width=10, int grid_height=10, float grid_resolution=0.05);
	~ThymarLidar(){delete mapper;};
	void processLidarMeasurement(const pcl::PCLPointCloud2ConstPtr& cloud_msg);
	void run();
       
};

ThymarLidar::ThymarLidar(int argc, char** argv, float hz, int grid_width, int grid_height, float grid_resolution)
	{
		
		// READ PARAMETERS
		ros::NodeHandle nh_private("~");
    	nh_private.getParam("name", this->name);
    	

		this->pointcloud_subscriber = this->nh.subscribe<pcl::PCLPointCloud2>("/" +this->name +"/velodyne_points", 5, &ThymarLidar::processLidarMeasurement, this);
        this->pointcloud_publisher = this->nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/" +this->name +"/world_map", 1);
        this->grid_publisher = this->nh.advertise<nav_msgs::OccupancyGrid>("/" +this->name +"/occupancy_grid", 1);

        this->grid.info.resolution = grid_resolution;
		this->grid.info.width = (int) std::round(grid_width/grid_resolution);
		this->grid.info.height = (int) std::round(grid_height/grid_resolution);

		this->mapper= new PointCloudMapper((int) std::round(grid_width/grid_resolution), (int) std::round(grid_height/grid_resolution), grid_resolution);

		std::cout << "Lidar PointCloud Processor node initialised for robot " << this->name << std::endl;
	


	}

void ThymarLidar::processLidarMeasurement(const pcl::PCLPointCloud2ConstPtr& cloud_msg){

		pcl::fromPCLPointCloud2(*cloud_msg, this->cloud);
		this->new_point_cloud = true;
	}

void ThymarLidar::run(){
	while(ros::ok()){

		if(this->new_point_cloud){
			this->new_point_cloud=false;
			this->mapper->addPointCloud(this->cloud);

			
			this->grid.data = this->mapper->getOccupancyGrid();
			this->grid_publisher.publish(this->grid);

			pcl::PointCloud<pcl::PointXYZ> world_map = this->mapper->getWorldPointCloud();
        	this->pointcloud_publisher.publish(world_map);
		}

		ros::spinOnce();
		this->rate.sleep();
	}
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "Lidar_PointCloud_Processor");
    ThymarLidar lidar = ThymarLidar(argc,argv);
    lidar.run();



    return 0;
}