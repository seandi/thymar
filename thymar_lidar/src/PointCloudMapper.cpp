#include "PointCloudMapper.hpp"

PointCloudMapper::PointCloudMapper(int grid_width, int grid_height, float grid_resolution){
	this->grid_width = grid_width;
	this->grid_height = grid_height;
	this->grid_resolution = grid_resolution;

	this->occupancy_grid.resize(grid_width*grid_height);
	std::fill(this->occupancy_grid.begin(),this->occupancy_grid.end(),-1);

	for(int i=0; i<this->occupancy_grid.size(); i++){

		int y = i%this->grid_width;
		int x = i/this->grid_width;
		float xf = (float) (x-this->grid_width/2.)*this->grid_resolution;
		float yf = (float) (y-this->grid_height/2.)*this->grid_resolution;

		
		if((pow(xf,2) + pow(yf,2)) < pow(0.45,2)){
			this->occupancy_grid[y*this->grid_width+x]=0;
		}
		
	}

	
}

void PointCloudMapper::addPointCloud(pcl::PointCloud<pcl::PointXYZ> new_point_cloud, Pose2d pose2d){
	pcl::PointCloud<pcl::PointXYZ> terrain_cloud; 
	std::vector<float> sphere_coeff;


	if(this->first){
		this->world_point_cloud=new_point_cloud;

		this->obstacles_point_cloud = this->directionalFilter(new_point_cloud, "z", -0.05, 5.0, terrain_cloud);
		this->terrain_point_cloud = terrain_cloud;


		this->first=false;
	}else{
		pcl::PointCloud<pcl::PointXYZ> rotated_new_point_cloud = this->transform(new_point_cloud, pose2d.x,pose2d.y,0.0,pose2d.theta);
		
		this->world_point_cloud+=rotated_new_point_cloud;

		pcl::PointCloud<pcl::PointXYZ> filtered_new_point_cloud = this->directionalFilter(rotated_new_point_cloud, "z", -0.05, 5.0, terrain_cloud);
		this->obstacles_point_cloud+=filtered_new_point_cloud;
		this->terrain_point_cloud+=terrain_cloud;
		
	}

	this->world_point_cloud = this->downSample(this->world_point_cloud.makeShared());
	this->obstacles_point_cloud = this->downSample(this->obstacles_point_cloud.makeShared());
    this->terrain_point_cloud = this->downSample(this->terrain_point_cloud.makeShared());

    if(!this->target_found){
	    sphere_coeff = this->locateSphere(this->obstacles_point_cloud);
	    if(sphere_coeff.size()>0){
	    	this->target_found=true;
	    	this->target_model.x = sphere_coeff[0];
	    	this->target_model.y = sphere_coeff[1];
	    	this->target_model.radius = sphere_coeff[3];
	    }
	}


	
	

	float x,y;
    for(int n=0; n<this->obstacles_point_cloud.points.size(); n++){
    	x = this->obstacles_point_cloud.points[n].x;
    	y = this->obstacles_point_cloud.points[n].y;

    	int xi = (int) std::round(x/grid_resolution);
    	int yi = (int) std::round(y/grid_resolution);

    	this->occupancy_grid[(yi+(this->grid_height/2))*this->grid_width+xi+(this->grid_width/2)] = 100;
    }

    
    for(int n=0; n<this->terrain_point_cloud.points.size(); n++){
    	x = this->terrain_point_cloud.points[n].x;
    	y = this->terrain_point_cloud.points[n].y;

    	int xi = (int) std::round(x/grid_resolution);
    	int yi = (int) std::round(y/grid_resolution);

    	int index = (yi+(this->grid_height/2))*this->grid_width+xi+(this->grid_width/2);
    	// Do not overwrite obstacles grids!
    	if(this->occupancy_grid[index] <= 0){
    		this->occupancy_grid[index] = 0;
    	}
    	
    }
}

pcl::PointCloud<pcl::PointXYZ> PointCloudMapper::downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud){
	pcl::PointCloud<pcl::PointXYZ> target_cloud;

	pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(source_cloud);
    sor.setLeafSize(0.05,0.05,0.05);
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

pcl::PointCloud<pcl::PointXYZ> PointCloudMapper::directionalFilter(pcl::PointCloud<pcl::PointXYZ> source_cloud, std::string axis,
 float lower_bound, float upper_bound,
  pcl::PointCloud<pcl::PointXYZ>& outliers_cloud)
{

	pcl::PointCloud<pcl::PointXYZ> filtered_cloud;

	pcl::PassThrough<pcl::PointXYZ> filter(true);
	filter.setInputCloud (source_cloud.makeShared());
	filter.setFilterFieldName (axis);
	filter.setFilterLimits (lower_bound,upper_bound);
	filter.filter (filtered_cloud);

	
	pcl::IndicesConstPtr indices = filter.getRemovedIndices();


	pcl::ExtractIndices<pcl::PointXYZ> extract;
 
	extract.setInputCloud (source_cloud.makeShared());
	extract.setIndices (indices);
	extract.filter (outliers_cloud);
	
	return filtered_cloud;
}


std::vector<float> PointCloudMapper::fitSphere(pcl::PointCloud<pcl::PointXYZ> input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr inliers_cloud){
	pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg;
	
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

	pcl::PointCloud<pcl::Normal> cloud_normals;
	pcl::PointIndices::Ptr model_inliers(new pcl::PointIndices);
	pcl::ModelCoefficients::Ptr coefficients_model (new pcl::ModelCoefficients);

	normal_estimator.setSearchMethod(tree);
	normal_estimator.setInputCloud(input_cloud.makeShared());
	normal_estimator.setKSearch (50);
	normal_estimator.compute (cloud_normals);


	seg.setOptimizeCoefficients (true);
	seg.setModelType (pcl::SACMODEL_SPHERE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setNormalDistanceWeight (0.1);
	seg.setMaxIterations (10000);
	seg.setDistanceThreshold (0.13);
	seg.setRadiusLimits (0.12, 0.17);
	seg.setInputCloud (input_cloud.makeShared());
	seg.setInputNormals (cloud_normals.makeShared());

	seg.segment (*model_inliers, *coefficients_model);
	
	if(inliers_cloud){
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		extract.setInputCloud (input_cloud.makeShared());
		extract.setIndices (model_inliers);
		extract.filter (*inliers_cloud);
	}
	int n_inliers = model_inliers->indices.size();
	std::cout << "N inliers: " << n_inliers << std::endl;
	std::cout << "coefficients: " << coefficients_model->values.size() << std::endl;

	return (n_inliers > 0) ? coefficients_model->values : std::vector<float>();
}

std::vector<float> PointCloudMapper::locateSphere(pcl::PointCloud<pcl::PointXYZ> input_cloud){
	
	pcl::search::KdTree<pcl::PointXYZ>::Ptr cluster_tree (new pcl::search::KdTree<pcl::PointXYZ>);
	cluster_tree->setInputCloud (input_cloud.makeShared());

	std::vector<pcl::PointIndices> cluster_indices;
	pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
	ec.setClusterTolerance (0.10); // 2cm
	ec.setMinClusterSize (40);
	ec.setMaxClusterSize (300);
	ec.setSearchMethod (cluster_tree);
	ec.setInputCloud (input_cloud.makeShared());
	ec.extract (cluster_indices);

	int j = 0;
	std::vector<float> sphere_coeff;
	bool found = false;
	for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
		  cloud_cluster->points.push_back (input_cloud.points[*pit]); //*
		
		cloud_cluster->width = cloud_cluster->points.size ();
		cloud_cluster->height = 1;
		cloud_cluster->is_dense = true;

		j++;
		std::cout << "PointCloud " << j << " representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
		if(!found){
			sphere_coeff = this->fitSphere(*cloud_cluster);
			if(sphere_coeff.size()>0){
				//return sphere_coeff;
				found=true;
			}
		}
		
	}

	return sphere_coeff;
}

