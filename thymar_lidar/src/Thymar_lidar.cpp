#include "ros/ros.h"
#include <iostream>
#include "pcl/point_cloud.h"
#include <pcl_ros/point_cloud.h>
#include "pcl_conversions/pcl_conversions.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <tf/tf.h>
#include <string>
#include "PointCloudMapper.hpp"




class ThymarLidar{
private:
	ros::NodeHandle nh;
	ros::Rate rate = ros::Rate(10.0);
    ros::Subscriber pointcloud_subscriber;
    ros::Publisher pointcloud_publisher;
    ros::Publisher grid_publisher;
    ros::Subscriber odometry_subscriber;
    ros::Publisher obstacles_publisher;
    ros::Publisher terrain_publisher;
    ros::Publisher target_marker_publisher;



    std::string name;
    PointCloudMapper* mapper;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    Pose2d pose2d;
    nav_msgs::OccupancyGrid grid;
    bool new_point_cloud = false;
    SphereModel target_model;
    bool target_found = false;

public:
	ThymarLidar(int argc, char** argv, float hz=10, int grid_width=10, int grid_height=10, float grid_resolution=0.05);
	~ThymarLidar(){delete mapper;};
	void processLidarMeasurement(const pcl::PCLPointCloud2ConstPtr& cloud_msg);
	void parseOdometry(const nav_msgs::Odometry::ConstPtr& msg);
	void publishTargetMarker();
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
        this->odometry_subscriber = this->nh.subscribe<nav_msgs::Odometry>("/" +this->name +"/odom", 5, &ThymarLidar::parseOdometry, this);

        this->obstacles_publisher = this->nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/" +this->name +"/obstacles", 1);
        this->terrain_publisher = this->nh.advertise<pcl::PointCloud<pcl::PointXYZ> > ("/" +this->name +"/traversable_terrain", 1);

        this->grid.info.resolution = grid_resolution;
		this->grid.info.width = (int) std::round(grid_width/grid_resolution);
		this->grid.info.height = (int) std::round(grid_height/grid_resolution);

		this->mapper= new PointCloudMapper((int) std::round(grid_width/grid_resolution), (int) std::round(grid_height/grid_resolution), grid_resolution);

		this->target_marker_publisher = this->nh.advertise<visualization_msgs::Marker>("/" +this->name +"/target_marker", 1);

		std::cout << "Lidar PointCloud Processor node initialised for robot " << this->name << std::endl;
	


	}

void ThymarLidar::processLidarMeasurement(const pcl::PCLPointCloud2ConstPtr& cloud_msg){

		pcl::fromPCLPointCloud2(*cloud_msg, this->cloud);
		this->new_point_cloud = true;
	}

void ThymarLidar::parseOdometry(const nav_msgs::Odometry::ConstPtr& msg){

		double x = msg->pose.pose.position.x;
		double y = msg->pose.pose.position.y;

		tf::Quaternion q(
			msg->pose.pose.orientation.x,
			msg->pose.pose.orientation.y,
			msg->pose.pose.orientation.z,
			msg->pose.pose.orientation.w
		);
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		this->pose2d.x = x;
		this->pose2d.y = y;
		this->pose2d.theta = yaw;
		
	}

void ThymarLidar::run(){
	while(ros::ok()){

		if(this->new_point_cloud){
			this->new_point_cloud=false;
			this->mapper->addPointCloud(this->cloud, this->pose2d);

			
			this->grid.data = this->mapper->getOccupancyGrid();
			this->grid_publisher.publish(this->grid);

			pcl::PointCloud<pcl::PointXYZ> world_map = this->mapper->getWorldPointCloud();
        	this->pointcloud_publisher.publish(world_map);

        	pcl::PointCloud<pcl::PointXYZ> obstacles_map = this->mapper->getObstaclesPointCloud();
        	this->obstacles_publisher.publish(obstacles_map);
        	pcl::PointCloud<pcl::PointXYZ> terrain_map = this->mapper->getTerrainPointCloud();
        	this->terrain_publisher.publish(terrain_map);

        	if(!this->target_found && this->mapper->isTargetFound()){
        		this->target_found = true;
        		this->target_model = this->mapper->getTargetModel();

        		std::cout << "Model found in (" << this->target_model.x << ","  << this->target_model.y << ") " 
				<< "with radius: "<< this->target_model.radius 
				<< std::endl;
				this->publishTargetMarker();

        	}
		}

		ros::spinOnce();
		this->rate.sleep();
	}
}

void ThymarLidar::publishTargetMarker(){
	visualization_msgs::Marker marker;
	
	marker.header.frame_id = "/thymar/base_link";
	//marker.header.stamp = ros::Time::now();

	marker.ns = "basic_shapes";
	marker.id = 4;

	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;

	marker.pose.position.x = this->target_model.x;
	marker.pose.position.y = this->target_model.y;
	marker.pose.position.z = this->target_model.radius;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = this->target_model.radius*2.0;
	marker.scale.y = this->target_model.radius*2.0;
	marker.scale.z = this->target_model.radius*2.0;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	marker.lifetime = ros::Duration(0.0);;

	this->target_marker_publisher.publish(marker);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "Lidar_PointCloud_Processor");
    ThymarLidar lidar = ThymarLidar(argc,argv);
    lidar.run();



    return 0;
}