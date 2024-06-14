// Copyright 2014 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
//#include "cloud_creator_align.hpp"
#include <pcl_lab/cloudMatcher.hpp>


using namespace std::chrono_literals;
using std::placeholders::_1;

CloudMatcher::CloudMatcher() : Node("pclsub")
{ 

    auto publish_AlignMap =
      [this]() -> void
    {   
        //publisher_SourcePointcloud_ ->publish(std::move(PointcloudSource_ros_));
        publisher_PointcloudMap_->publish(std::move(PointcloudMap_ros_));
    };
   
    declare_parameter("pointcloud_map","");
    declare_parameter("input_pointcloud","");
    declare_parameter("input_odom","");
    declare_parameter("input_imu","");
    declare_parameter("threshold_align_fitness_score",0.5);
    declare_parameter("save_pcd_dir","");
    declare_parameter("threshold_dist_save_pcd",5.0);
    declare_parameter("Submap_Range",60.0);
    declare_parameter("SumAlign_fitnessScore",500.0);
    declare_parameter("leaf_size",0.5);
    declare_parameter("threshold_score_align_with_map",1.0);
    declare_parameter("threshold_adjust_pose",-2.2);
    declare_parameter("threshold_EKF_align_fitness_score",0.5);
   
    declare_parameter("GICP_TransformationEpsilon",0.1);
    declare_parameter("GICP_EuclideanFitnessEpsilon",0.1);
    declare_parameter("GICP_MaximumIterations",30);

    declare_parameter("NDT_OMP_TransformationEpsilon", 0.1);
    declare_parameter("NDT_OMP_StepSize",0.75);
    declare_parameter("NDT_OMP_Resolution", 5.0);
    declare_parameter("NDT_OMP_MaximumIterations", 30);

    str_pointcloud_map_     = this->get_parameter("pointcloud_map").as_string();
    str_source_pointcloud_  = this->get_parameter("input_pointcloud").as_string();
    str_input_odom_         = this->get_parameter("input_odom").as_string();
    str_input_imu_          = this->get_parameter("input_imu").as_string();
    threshold_align_fitness_score_= this->get_parameter("threshold_align_fitness_score").as_double();
    str_save_pcd_dir_       = this->get_parameter("save_pcd_dir").as_string();
    threshold_dist_save_pcd_= this->get_parameter("threshold_dist_save_pcd").as_double();
    SubmapRange_            = this->get_parameter("Submap_Range").as_double();
    SumAlignFitnessScore_   = this->get_parameter("SumAlign_fitnessScore").as_double();
    leafSize_               = this->get_parameter("leaf_size").as_double();
    threshold_score_align_with_map_ = this->get_parameter("threshold_score_align_with_map").as_double();
    threshold_adjust_pose_  = this->get_parameter("threshold_adjust_pose").as_double();
    threshold_EKF_align_fitness_score_  = this->get_parameter("threshold_EKF_align_fitness_score").as_double();

    GICP_TransformationEpsilon_ = this->get_parameter("GICP_TransformationEpsilon").as_double();
    GICP_EuclideanFitnessEpsilon_ = this->get_parameter("GICP_EuclideanFitnessEpsilon").as_double();
    GICP_MaximumIterations_ = this->get_parameter("GICP_MaximumIterations").as_int();
    
    NDT_OMP_TransformationEpsilon_  = this->get_parameter("NDT_OMP_TransformationEpsilon").as_double();//(0.1);
    NDT_OMP_StepSize_               = this->get_parameter("NDT_OMP_StepSize").as_double();//(0.75); //<- これかも
    NDT_OMP_Resolution_             = this->get_parameter("NDT_OMP_Resolution").as_double();//(5.0);
    NDT_OMP_MaximumIterations_      = this->get_parameter("NDT_OMP_MaximumIterations").as_int();//(30);

    printf("pointcloud_map=%s \n",str_pointcloud_map_.c_str());
    printf("source_pointcloud=%s \n",str_source_pointcloud_.c_str());
    printf("input_odom=%s \n",str_input_odom_.c_str());
    printf("threshold_align_fitness_score=%lf \n",threshold_align_fitness_score_);
    printf("str_save_pcd_dir_=%s \n",str_save_pcd_dir_.c_str());
    printf("threshold_dist_save_pcd =%lf \n",threshold_dist_save_pcd_);
    printf("Submap_Range =%lf \n",SubmapRange_);
    printf("SumAlign_FitnessScore =%lf \n",SumAlignFitnessScore_);
    printf("threshold_score_align_with_map =%lf \n",threshold_score_align_with_map_);
    printf("threshold_adjust_pose =%lf \n",threshold_adjust_pose_);
    printf("threshold_EKF_align_fitness_score =%lf \n",threshold_EKF_align_fitness_score_);

    printf("GICP_TransformationEpsilon =%lf \n",GICP_TransformationEpsilon_);
    printf("GICP_EuclideanFitnessEpsilon =%lf \n",GICP_EuclideanFitnessEpsilon_);
    printf("GICP_MaximumIterations =%d \n",GICP_MaximumIterations_);
    
    printf("NDT_OMP_TransformationEpsilon =%lf \n",NDT_OMP_TransformationEpsilon_);
    printf("NDT_OMP_StepSize=%lf \n",NDT_OMP_StepSize_);
    printf("NDT_OMP_Resolution =%lf \n",NDT_OMP_Resolution_);
    printf("NDT_OMP_MaximumIterations =%d \n",NDT_OMP_MaximumIterations_);


    //poseOnOdom_.header.stamp = 0;
    subscriberLidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        str_source_pointcloud_.c_str(), 10, std::bind(&CloudMatcher::lidar_callback, this, _1));

    subscriberOdom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        str_input_odom_.c_str(), 10, std::bind(&CloudMatcher::odom_callback, this, std::placeholders::_1));

    subscriberImu_ = this->create_subscription<sensor_msgs::msg::Imu>(
        str_input_imu_.c_str(), 10, std::bind(&CloudMatcher::imu_callback, this, std::placeholders::_1));

    publisher_SourcePointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/source_pointcloud", 1);
    publisher_TargetPointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/target_pointcloud", 1);
    publisher_AlignPointcloud_  = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aligned_pointcloud", 10);
    publisher_PointcloudMap_    = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_pointcloud", 10);
    publisher_PointcloudMapAlign_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_aligned_pointcloud", 10);
    publisher_PointcloudMapAlignSum_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sum_aligned_pointcloud", 10);
    
    publisher_PredictPoseVis_   = this->create_publisher<visualization_msgs::msg::Marker>("/Predist_track", 10);
    publisher_EstimatePoseVis_  = this->create_publisher<visualization_msgs::msg::Marker>("/Estimate_track", 10);
    publisher_MapPoseVis_  = this->create_publisher<visualization_msgs::msg::Marker>("/ScanMatching_track", 10);

    publisher_PoseErrorCov_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/pose_error_cov", 10);

    publisher_PointcloudMapMatching_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/mapmatching_pointcloud", 1);

    timer_ = create_wall_timer(100ms, publish_AlignMap);

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    load_pointcloud(str_pointcloud_map_.c_str());

    count_RosbagPlay_ = -100;

    current_timeOdom_ = 0;
    dx_ = 0;
    dy_ = 0;
    dyaw_ = 0;
    
    Xpose_ = XposeMap_ = 0;
    Ypose_ = YposeMap_ = 0;
    Yawpose_ = YawposeMap_ = 0;

    dxy_mh_ = 0;

    qAdjust_ = 0;

    std::string str_pose_data = str_save_pcd_dir_;
    str_pose_data += "pose.dat";

    std::string str_odom_data = str_save_pcd_dir_;
    str_odom_data += "odom.dat";

    std::string str_ekf_data = str_save_pcd_dir_;
    str_ekf_data += "ekf.dat";

    f_pose_ = fopen(str_pose_data.c_str(),"w");
    f_odom_ = fopen( str_odom_data.c_str(),"w");
    f_ekf_  = fopen( str_ekf_data.c_str(),"w");

    std::cout << "--- pcl::GICP ---" << std::endl;
    boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>> gicp(new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());
    ptr_gicp_ = gicp;   

    std::cout << "--- pcl::NDTOMP ---" << std::endl;
    //pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>()); //ndt_omp->setResolution(5.0);
 
    boost::shared_ptr<pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>> ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
    ptr_ndt_omp_ = ndt_omp;   

    //boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>> gicp_omp(new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>()); 
    //ptr_gicp_omp_ = gicp_omp;
    
    //     // Default values
    // static int max_iter = 30;        // Maximum iterations
    // static float ndt_res = 1.0;      // Resolution
    // static double step_size = 0.1;   // Step size
    // static double trans_eps = 0.01;  // Transformation epsilon

    // // Leaf size of VoxelGrid filter.
    // static double voxel_leaf_size = 2.0;

        // icp.setMaxCorrespondenceDistance(100);
        // icp.setMaximumIterations(100);
        // icp.setTransformationEpsilon(1e-6);
        // icp.setEuclideanFitnessEpsilon(1e-6);
        // icp.setRANSACIterations(0);

        //ndt.setTransformationEpsilon(0.01);
        //ndt.setResolution(1.0);
    
    // ptr_gicp_->setMaximumIterations(50);
    // ptr_gicp_->setRANSACIterations(30);
    // ptr_gicp_->setEuclideanFitnessEpsilon(1e-6);
    // //ptr_gicp_->setTransformationEstimation(1e-2);
    // ptr_gicp_->setTranslationGradientTolerance(0.1);

    ptr_gicp_->setTransformationEpsilon(GICP_TransformationEpsilon_);
    ptr_gicp_->setEuclideanFitnessEpsilon(GICP_EuclideanFitnessEpsilon_);
    ptr_gicp_->setMaximumIterations(GICP_MaximumIterations_);    

    //From Autoware
    // ptr_ndt_omp_->setTransformationEpsilon(0.1);
    // ptr_ndt_omp_->setStepSize(0.1);
    // ptr_ndt_omp_->setResolution(5.0);
        
    //ptr_ndt_omp_->setEuclideanFitnessEpsilon(0.01); //NG
    //ptr_ndt_omp_->setEuclideanFitnessEpsilon(1e-8); 
    //ptr_ndt_omp_->setTransformationEstimation();
    
    ptr_ndt_omp_->setTransformationEpsilon(NDT_OMP_TransformationEpsilon_);
    ptr_ndt_omp_->setStepSize(NDT_OMP_StepSize_); //<- これかも
    ptr_ndt_omp_->setResolution( NDT_OMP_Resolution_);
    ptr_ndt_omp_->setMaximumIterations(NDT_OMP_MaximumIterations_);


    ptr_ndt_omp_->setNumThreads(omp_get_max_threads());
    ptr_ndt_omp_->setNeighborhoodSearchMethod(pclomp::DIRECT7);
    
    //ptr_ndt_omp_->setOutlierRatio(0.1);
    //ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT1);
    //ndt_omp->setEuclideanFitnessEpsilon(1e-6);
    //ndt_omp->setTransformationRotationEpsilon(1e-6);
    //ndt_omp->setResolution(leafSize_);
  
    
    EKF_init();

    //-- visualization on rviz
    PrePoseVis_.id = 0;
    EstPoseVis_.id = 1;
    MapPoseVis_.id = 2;

    PrePoseVis_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    EstPoseVis_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    MapPoseVis_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    
    PrePoseVis_.scale.x = PrePoseVis_.scale.y = 1.0;
    EstPoseVis_.scale.x = EstPoseVis_.scale.y = 0.8;
    MapPoseVis_.scale.x = MapPoseVis_.scale.y = 1.0;

    //Orange
    PrePoseVis_.color.r = 1.0f; //FF
    PrePoseVis_.color.g = 0.65f; //A5
    PrePoseVis_.color.b = 0.0f; //00
    PrePoseVis_.color.a = 1.0; 

    //Green
    EstPoseVis_.color.r = 0.0f; //FF
    EstPoseVis_.color.g = 1.0f; //A5
    EstPoseVis_.color.b = 0.0f; //00
    EstPoseVis_.color.a = 1.0; 
    
    //Blue
    MapPoseVis_.color.r = 0.3f; //FF
    MapPoseVis_.color.g = 0.25f; //A5
    MapPoseVis_.color.b = 1.0f; //00
    MapPoseVis_.color.a = 1.0; 

    PrePoseVis_.points.clear();
    EstPoseVis_.points.clear();
    MapPoseVis_.points.clear();

    PrePoseVis_.header.stamp = this->now();
    EstPoseVis_.header.stamp = this->now();
    MapPoseVis_.header.stamp = this->now();

    PrePoseVis_.pose.position.x = PrePoseVis_.pose.position.y = PrePoseVis_.pose.position.z = 0;
    EstPoseVis_.pose.position.x = EstPoseVis_.pose.position.y = EstPoseVis_.pose.position.z = 0;
    MapPoseVis_.pose.position.x = MapPoseVis_.pose.position.y = MapPoseVis_.pose.position.z = 0;

    PrePoseVis_.pose.orientation.x = 0;
    PrePoseVis_.pose.orientation.y = 0;
    PrePoseVis_.pose.orientation.z = 0;
    PrePoseVis_.pose.orientation.w = 0;

    EstPoseVis_.pose.orientation.x = 0;
    EstPoseVis_.pose.orientation.y = 0;
    EstPoseVis_.pose.orientation.z = 0;
    EstPoseVis_.pose.orientation.w = 0;

    MapPoseVis_.pose.orientation.x = 0;
    MapPoseVis_.pose.orientation.y = 0;
    MapPoseVis_.pose.orientation.z = 0;
    MapPoseVis_.pose.orientation.w = 0;

    PrePoseVis_.header.frame_id ="base_link";
    PrePoseVis_.action = visualization_msgs::msg::Marker::ADD;

    EstPoseVis_.header.frame_id ="base_link";
    EstPoseVis_.action = visualization_msgs::msg::Marker::ADD;

    MapPoseVis_.header.frame_id ="base_link";
    MapPoseVis_.action = visualization_msgs::msg::Marker::ADD;
}

void CloudMatcher::load_pointcloud(const std::string &file_name){

    // PointCloudMapを読み込む
    if(pcl::io::loadPCDFile(file_name, PointcloudMap_pcl_) == 0){
           
        pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::VoxelGrid<pcl::PointXYZ> voxelgrid;
        
        voxelgrid.setLeafSize(leafSize_, leafSize_, leafSize_);
        //voxelgrid.setFilterFieldName("z");  // Z軸（高さ）の値でフィルタをかける
        //voxelgrid.setFilterLimits(0.1, 10.0);  // 0.1 ～ 1.0 m の間にある点群を抽出

        voxelgrid.setInputCloud(PointcloudMap_pcl_.makeShared());
        voxelgrid.filter(*downsampled);
        PointcloudMap_pcl_ = *downsampled;

        pcl::toROSMsg(PointcloudMap_pcl_, PointcloudMap_ros_);
        PointcloudMap_ros_.header.frame_id = "base_link";
        
        is_loadPCDFile_ = true;
    }
    else{
        is_loadPCDFile_ = false;
    }
}

bool CloudMatcher::Is_LoadPCDFile(void){
    return is_loadPCDFile_;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudMatcher::align(boost::shared_ptr<pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>> registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud ) {

    /*initial guess*/
	Eigen::Translation3f init_translation(
        (float)transform_ros_.position.x,
        (float)transform_ros_.position.y,
        (float)transform_ros_.position.z
    );
	Eigen::AngleAxisf init_rotation(
		QuatMsgToEigen(transform_ros_.orientation)
    );

    std::cout << "init_translation = (" << init_translation.x() << ", " << init_translation.y() << ", " << init_translation.z() << ")" << std::endl; 
	std::cout << "init_rotation : (" << init_rotation.axis()(0) << ", " << init_rotation.axis()(1) << ", " << init_rotation.axis()(2) << "), " << init_rotation.angle() << " [rad]" << std::endl; 
	Eigen::Matrix4f init_guess = (init_translation*init_rotation).matrix();


    registration->setInputTarget(target_cloud);
    registration->setInputSource(source_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());

    rclcpp::Clock system_clock;

    ///auto t1 = ros::WallTime::now();
    auto t1 = system_clock.now();
    registration->align(*aligned,init_guess);
    
    align_fitnessScore_ = registration->getFitnessScore();

    auto t2 = system_clock.now();
    std::cout << "single : " << (t2 - t1).seconds()* 1000 << "[msec]" << std::endl;
    std::cout << "fitness: " << registration->getFitnessScore() << std::endl << std::endl;
    transform_eigen_ = registration->getFinalTransformation().cast<double>();

    return aligned;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr CloudMatcher::align_wo_init_trans(boost::shared_ptr<pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>> registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud ) {

    registration->setInputTarget(target_cloud);
    registration->setInputSource(source_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr aligned(new pcl::PointCloud<pcl::PointXYZ>());

    rclcpp::Clock system_clock;

    auto t1 = system_clock.now();
    registration->align(*aligned);
    
    align_fitnessScore_ = registration->getFitnessScore();

    auto t2 = system_clock.now();
    std::cout << "single : " << (t2 - t1).seconds()* 1000 << "[msec]" << std::endl;
    std::cout << "fitness: " << registration->getFitnessScore() << std::endl << std::endl;

    return aligned;
}



void CloudMatcher::convertTransformEigenToROS(void)
{
       
    // convert Matrix4d to Affine3d for ROS conversion
    Eigen::Affine3d transform_affine_eigen;
   
    transform_affine_eigen = transform_eigen_;
    transform_ros_ = tf2::toMsg(transform_affine_eigen);

    tf2::Quaternion q(
        transform_ros_.orientation.x,
        transform_ros_.orientation.y,
        transform_ros_.orientation.z,
        transform_ros_.orientation.w);
    
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    dx_ = transform_ros_.position.x;
    dy_ = transform_ros_.position.y;
    dz_ = transform_ros_.position.z;
    
    yaw = -yaw;
    dyaw_ = yaw;
    dpitch_ = pitch;
    droll_ = roll;

    dd_ = sqrt(dx_*dx_ + dy_*dy_ + dz_*dz_);
    
    if(dtLidar_ > 0){
        vLidar_ = dd_/dtLidar_;
        wLidar_ = dyaw_/dtLidar_;
    } 

    //-- Lidar odometry
    Yawpose_    += yaw;
    Yawpose_    = modAngle(Yawpose_);
    //Pitchpose_  = -pitch;
    Pitchpose_  = pitch;
    Rollpose_   = -roll;
 
    Xpose_ += dd_*cos(Pitchpose_)*cos(Yawpose_);
    Ypose_ += dd_*cos(Pitchpose_)*sin(Yawpose_);
    
    //Zpose_ += dd_*sin(Pitchpose_);
    Zpose_  = dz_;

    // double cos_yaw = cos(Yawpose_);
    // double sin_yaw = sin(Yawpose_);

    // Xpose_ += cos_yaw*dx_ - sin_yaw*dy_;
    // Ypose_ += sin_yaw*dx_ + cos_yaw*dy_;
    // Zpose_  = dz_;
    
    //-- Scan Matching

    YawposeMap_    += yaw;
    YawposeMap_    = modAngle(YawposeMap_);
    //PitchposeMap_  += -pitch;//Pitchpose_;
    //PitchposeMap_  = pitch;//Pitchpose_;
    //PitchposeMap_  = PitchImu_;
    PitchposeMap_  = Pitchpose_;
    
    RollposeMap_   = Rollpose_;

    //PitchposeMap_  = PitchImu_;
    //RollposeMap_   = -RollImu_;

 
    XposeMap_ += dd_*cos(PitchposeMap_)*cos(YawposeMap_);
    YposeMap_ += dd_*cos(PitchposeMap_)*sin(YawposeMap_);
    
    // cos_yaw = cos(YawposeMap_);
    // sin_yaw = sin(YawposeMap_);

    // XposeMap_ += cos_yaw*dx_ - sin_yaw*dy_;
    // YposeMap_ += sin_yaw*dx_ + cos_yaw*dy_;
    // ZposeMap_  = dz_;
    
    ZposeMap_ = Zpose_;
    //ZposeMap_ += dd_*sin(PitchposeMap_);
    //ZposeMap_ = dz_;


    printf("align:: dd =%lf x:%lf, y:%lf, z:%lf, roll:%lf, pitch:%lf, yaw:%lf\n",
        dd_,
        transform_ros_.position.x,
        transform_ros_.position.y,
        transform_ros_.position.z,
        roll*180.0/M_PI,pitch*180.0/M_PI,yaw*180.0/M_PI);
  
    tf2::Quaternion Quat;
    Quat.setRPY( Rollpose_, Pitchpose_, Yawpose_);

    poseOnAlign_.pose.pose.position.x = Xpose_;
    poseOnAlign_.pose.pose.position.y = Ypose_;
    poseOnAlign_.pose.pose.position.z = Zpose_;
    
    poseOnAlign_.pose.pose.orientation.w = Quat.getW();
    poseOnAlign_.pose.pose.orientation.x = Quat.getX();    
    poseOnAlign_.pose.pose.orientation.y = Quat.getY();    
    poseOnAlign_.pose.pose.orientation.z = Quat.getZ();

}

void CloudMatcher::convertTransformEigenToROS_MapMatching(void)
{
     
    // convert Matrix4d to Affine3d for ROS conversion
    Eigen::Affine3d transform_affine_eigen;   
    transform_affine_eigen = transform_eigen_;
    transform_ros_ = tf2::toMsg(transform_affine_eigen);

    tf2::Quaternion q(
        transform_ros_.orientation.x,
        transform_ros_.orientation.y,
        transform_ros_.orientation.z,
        transform_ros_.orientation.w);
    
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    dx_ = transform_ros_.position.x;
    dy_ = transform_ros_.position.y;
    dz_ = transform_ros_.position.z;

    ddMap_ = sqrt(dx_*dx_ + dy_*dy_ + dz_*dz_);

    //yaw = -yaw;
    dyaw_ = yaw;
    dpitch_ = pitch;
    droll_ = roll;
    
    double cos_dyaw = cos(dyaw_);
    double sin_dyaw = sin(dyaw_);

    //if(vOdom_ ==0 || fabs(vLidar_) < 2.0*fabs(vOdom_)){
        // XposeMap_ += cos_dyaw*dx_ - sin_dyaw*dy_;
        // YposeMap_ += sin_dyaw*dx_ + cos_dyaw*dy_;
        // YawposeMap_ += yaw;
        // YawposeMap_ = modAngle(YawposeMap_);
        //ZposeMap_ += dd*sin(dpitch_);

        Xpose_ += cos_dyaw*dx_ - sin_dyaw*dy_;
        Ypose_ += sin_dyaw*dx_ + cos_dyaw*dy_;
        Yawpose_ += yaw;
        Yawpose_ = modAngle(Yawpose_);
        Zpose_ += dz_;

        printf("Map align:: dd =%lf x:%lf, y:%lf, z:%lf, roll:%lf, pitch:%lf, yaw:%lf\n",
            ddMap_ ,
            transform_ros_.position.x,
            transform_ros_.position.y,
            transform_ros_.position.z,
            roll*180.0/M_PI,pitch*180.0/M_PI,yaw*180.0/M_PI);
  
        tf2::Quaternion Quat;
        //Quat.setRPY( RollposeMap_, PitchposeMap_, YawposeMap_);
        Quat.setRPY( Rollpose_, Pitchpose_, Yawpose_);

        // poseOnAlign_.pose.pose.position.x = XposeMap_;
        // poseOnAlign_.pose.pose.position.y = YposeMap_;
        // poseOnAlign_.pose.pose.position.z = ZposeMap_;

        poseOnAlign_.pose.pose.position.x = Xpose_;
        poseOnAlign_.pose.pose.position.y = Ypose_;
        poseOnAlign_.pose.pose.position.z = Zpose_;

        poseOnAlign_.pose.pose.orientation.w = Quat.getW();
        poseOnAlign_.pose.pose.orientation.x = Quat.getX();    
        poseOnAlign_.pose.pose.orientation.y = Quat.getY();    
        poseOnAlign_.pose.pose.orientation.z = Quat.getZ();
    //}
}


void CloudMatcher::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    static unsigned int n=0 ;
    static int num=0;
    double lidar_align_fitnessScore = 0;

    pcl::fromROSMsg(*cloud_msg, PointcloudRaw_pcl_); 
    PointcloudRaw_pcl_.header.frame_id = "lidar";

    
    current_timeLidar_ = (double)(cloud_msg->header.stamp.sec + RCL_NS_TO_MS(cloud_msg->header.stamp.nanosec)*0.001);
    static double prev_time = current_timeLidar_;

    dtLidar_ = current_timeLidar_ - prev_time;

    // Lidar Odometry
    if(PointcloudTarget_pcl_.size () > 0 && PointcloudTarget_ros_.data.size () > 0){
        PointcloudSource_pcl_ = PointcloudTarget_pcl_;
        PointcloudSource_ros_ = PointcloudTarget_ros_;
    }

    pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
    voxelSampler.setInputCloud(PointcloudRaw_pcl_.makeShared());
    voxelSampler.setLeafSize(leafSize_, leafSize_ , leafSize_);
    // voxelSampler.setLeafSize(0.5, 0.5, 0.5);
    // voxelSampler.setFilterFieldName("z");  // Z軸（高さ）の値でフィルタをかける
    // voxelSampler.setFilterLimits(0.1, 50.0);  // 0.1 ～ 50.0 m の間にある点群を抽出
    voxelSampler.filter(PointcloudTarget_pcl_);

    pcl::toROSMsg(PointcloudTarget_pcl_, PointcloudTarget_ros_); 

    publisher_SourcePointcloud_ ->publish(std::move(PointcloudSource_ros_));
    publisher_TargetPointcloud_ ->publish(std::move(PointcloudTarget_ros_));

    if(PointcloudTarget_pcl_.size () > 0 && PointcloudSource_pcl_.size () > 0){

        //Step1: Lidar Odometry    
        printf("--- Step1: Lidar Odometry -------------- \n");
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_lidarOdom = align(ptr_gicp_, PointcloudTarget_pcl_.makeShared(),  PointcloudSource_pcl_.makeShared());
        //pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_lidarOdom = align(ptr_gicp_omp_, PointcloudTarget_pcl_.makeShared(),  PointcloudSource_pcl_.makeShared());
        //pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_lidarOdom = align(ptr_ndt_omp_, PointcloudTarget_pcl_.makeShared(),  PointcloudSource_pcl_.makeShared());

        convertTransformEigenToROS();
        

        qAdjust_ = YawposeMap_ - qOdom_;

        EKF_predict();
        eval_sigma_ = log(sqrt(postPt_(0,0)*postPt_(0,0) + postPt_(1,1)*postPt_(1,1) + postPt_(2,2)*postPt_(2,2)) / 3.0);
        
        //Eigen::Affine3f transformation;
        // if(eval_sigma_ < 0.11){
        //     Xpose_=XposeMap_; 
        //     Ypose_=YposeMap_; 
        //     Zpose_=ZposeMap_; 
        //     Rollpose_=RollposeMap_; 
        //     Pitchpose_=PitchposeMap_;
        //     Yawpose_ =YawposeMap_;
        // }
        
        Eigen::Affine3f transformation = pcl::getTransformation(Xpose_, Ypose_, Zpose_, Rollpose_, Pitchpose_, Yawpose_);  
        //Eigen::Affine3f transformation = pcl::getTransformation(x_pre_(0), x_pre_(1), Zpose_, Rollpose_, Pitchpose_, x_pre_(2));  
        //Eigen::Affine3f transformation = pcl::getTransformation(x_pre_(0), x_pre_(1), Zpose_, Rollpose_, Pitchpose_, Yawpose_);  
        
        //Eigen::Affine3f transformation = pcl::getTransformation(XposeMap_, YposeMap_, ZposeMap_, RollposeMap_, PitchposeMap_, YawposeMap_);  
        
        pcl::transformPointCloud(*aligned_lidarOdom, PointcloudAlign_pcl_, transformation);
        pcl::toROSMsg(PointcloudAlign_pcl_, PointcloudAlign_ros_); 
     
        lidar_align_fitnessScore = align_fitnessScore_;

        //Step2: Scan matching with map    
        //if(num == 0 || lidar_align_fitnessScore < threshold_align_fitness_score_ ){
        //if(lidar_align_fitnessScore < threshold_align_fitness_score_ ){
            
        printf("--- Step2: Scan matching with map : %lf / %lf -------------- \n", lidar_align_fitnessScore, threshold_align_fitness_score_);

        /*            
        voxelSampler.setInputCloud(PointcloudAlign_pcl_.makeShared());
        voxelSampler.setLeafSize(leafSize_, leafSize_ , leafSize_);
        voxelSampler.filter(PointcloudAlignVoxel_pcl_);
            
        Eigen::Affine3f transformation = pcl::getTransformation(0,0,0,0,-PitchposeMap_,0);  
        // pcl::transformPointCloud(PointcloudAlign_pcl_, PointcloudAlign_pcl_, transformation);
        // pcl::toROSMsg(PointcloudAlign_pcl_, PointcloudAlign_ros_); 

        pcl::transformPointCloud(PointcloudAlignVoxel_pcl_, PointcloudAlignVoxel_pcl_, transformation);
        pcl::toROSMsg(PointcloudAlignVoxel_pcl_, PointcloudAlignVoxel_ros_); 
        */

        //Eigen::Affine3f transformation = pcl::getTransformation(0,0,0,0,PitchposeMap_,0);  
        //Eigen::Affine3f transformation2 = pcl::getTransformation(0,0,0,0,PitchImu_,0);  
            
        //pcl::transformPointCloud(PointcloudAlign_pcl_, PointcloudAlign_pcl_, transformation2);
        //pcl::toROSMsg(PointcloudAlign_pcl_, PointcloudAlign_ros_); 

        //pcl::transformPointCloud(PointcloudAlignVoxel_pcl_, PointcloudAlignVoxel_pcl_, transformation);
        //pcl::toROSMsg(PointcloudAlignVoxel_pcl_, PointcloudAlign_ros_); 

        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_mapmatching = align(ptr_ndt_omp_, PointcloudMap_pcl_.makeShared(), PointcloudAlign_pcl_.makeShared());
        //pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_mapmatching = align(ptr_gicp_, PointcloudMap_pcl_.makeShared(), PointcloudAlign_pcl_.makeShared());
            
            
        //pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_mapmatching = align(ptr_gicp_, PointcloudMap_pcl_.makeShared(), PointcloudAlignVoxel_pcl_.makeShared());
        //pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_mapmatching = align(ptr_ndt_omp_, PointcloudMap_pcl_.makeShared(), PointcloudAlignVoxel_pcl_.makeShared());
        //pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_mapmatching = align(ptr_gicp_omp_, PointcloudMap_pcl_.makeShared(), PointcloudAlignVoxel_pcl_.makeShared());
            
        printf("Scan matching::align_fitnessScore_ = %lf / %lf -------------- \n", align_fitnessScore_, threshold_score_align_with_map_);

        fprintf(f_pose_,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
            current_timeOdom_ ,
            Xpose_,
            Ypose_, 
            Zpose_,
            XposeMap_,
            YposeMap_, 
            ZposeMap_,
            YawposeMap_*180/M_PI,
            PitchposeMap_*180/M_PI,
            RollposeMap_*180/M_PI,
            align_fitnessScore_,
            PitchImu_*180/M_PI,
            RollImu_*180/M_PI,                
            vLidar_,
            vOdom_,
            dtLidar_,
            dtOdom_,
            lidar_align_fitnessScore,
            dx_,//19
            dy_,//20
            dz_,//21
            dyaw_);  

        if(align_fitnessScore_ < threshold_score_align_with_map_){
            convertTransformEigenToROS_MapMatching();
               
            //if(align_fitnessScore_ <= threshold_EKF_align_fitness_score_){
                EKF_update();
            //}

            geometry_msgs::msg::Point estimate_point_geometry;
            
            estimate_point_geometry.x = -x_est_(0);
            estimate_point_geometry.y = -x_est_(1);
            estimate_point_geometry.z =  Zpose_;

            EstPoseVis_.header.stamp = this->now();
            EstPoseVis_.points.push_back(estimate_point_geometry);            
            publisher_EstimatePoseVis_->publish(std::move(EstPoseVis_));

            //qAdjust_ = Yawpose_ - qOdom_;

            //Eigen::Affine3f transformatoin = pcl::getTransformation(-dx_, -dy_, -dz_, -droll_, -dpitch_, -dyaw_);	        
            //Eigen::Affine3f transformatoin = pcl::getTransformation(dx_, dy_, dz_, droll_, dpitch_, dyaw_);	        
            
                //pcl::transformPointCloud(PointcloudAlign_pcl_, PointcloudAlign_pcl_, transformatoin);
                //pcl::toROSMsg(PointcloudAlign_pcl_, PointcloudAlign_ros_);     
            
                pcl::toROSMsg(*aligned_mapmatching, PointcloudMapMatching_ros_); 
                publisher_PointcloudMapMatching_->publish(std::move(PointcloudMapMatching_ros_));
        }
        //}

        if(n == 0 || (n > 5 && n%5 == 0)){
            geometry_msgs::msg::Point predict_point_geometry;
           
            // predict_point_geometry.x = -Xpose_;
            // predict_point_geometry.y = -Ypose_;
            
            // predict_point_geometry.x = Xpose_;
            // predict_point_geometry.y = Ypose_;
            //predict_point_geometry.z = Zpose_;
            
             predict_point_geometry.x = -x_pre_(0);
             predict_point_geometry.y = -x_pre_(1);
             predict_point_geometry.z =  Zpose_;

            PrePoseVis_.header.stamp = this->now();
            PrePoseVis_.points.push_back(predict_point_geometry);            
            publisher_PredictPoseVis_->publish(std::move(PrePoseVis_));


            geometry_msgs::msg::Point scanmatching_point_geometry;

            MapPoseVis_.header.stamp = this->now();
            MapPoseVis_.points.push_back(scanmatching_point_geometry);

            // scanmatching_point_geometry.x = -XposeMap_;
            // scanmatching_point_geometry.y = -YposeMap_;
            // scanmatching_point_geometry.z = ZposeMap_;

            scanmatching_point_geometry.x = -Xpose_;
            scanmatching_point_geometry.y = -Ypose_;
            scanmatching_point_geometry.z = Zpose_;

            MapPoseVis_.header.stamp = this->now();
            MapPoseVis_.points.push_back(scanmatching_point_geometry);  
            publisher_MapPoseVis_->publish(std::move(MapPoseVis_));

            // EstPoseVis_.header.stamp = this->now();
            // EstPoseVis_.points.push_back(estimate_point_geometry);            
            // publisher_EstimatePoseVis_->publish(std::move(EstPoseVis_));

        }
        n++;

        // Xpose_  = x_pre_(0);
        // Ypose_  = x_pre_(1);
        // Yawpose_= x_pre_(2);   
        
        pcl::toROSMsg(PointcloudAlign_pcl_, PointcloudAlign_ros_);  
        publisher_AlignPointcloud_ ->publish(std::move(PointcloudAlign_ros_));
        
            
        broadcastTF();
    }

     fprintf(f_ekf_,"%lf  %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
            current_timeLidar_,
            dtOdom_,
            vOdom_,
            wOdom_,
            x_pre_(0),
            x_pre_(1), 
            x_pre_(2),
            x_est_(0),
            x_est_(1),
            x_est_(2),
            eval_sigma_,
            dxy_mh_,
            prePt_(0,0),
            prePt_(1,1),
            prePt_(2,2),
            ddMap_,
            dx_,
            dy_,
            dz_,
            dyaw_
            );  


    eval_sigma_ = 0.0;    
    prev_time = current_timeLidar_;
    count_RosbagPlay_ = 0;
}

void CloudMatcher::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    prev_poseOnOdom_ = poseOnOdom_;
    
    poseOnOdom_ = *msg;
    poseOnOdom_.header.frame_id = "base_link";

    //convert ROS1 bag to ROS2
    static double prev_time = -1.0;
    static double l_vOdom = 0;
    current_timeOdom_ = (double)(msg->header.stamp.sec + RCL_NS_TO_MS(msg->header.stamp.nanosec)*0.001);

    printf("Odom::%lf::x=%lf,y=%lf\n",current_timeOdom_,msg->pose.pose.position.x,msg->pose.pose.position.y);
    printf("Twist::%lf::v=%lf,w=%lf\n",current_timeOdom_,msg->twist.twist.linear.x,msg->twist.twist.angular.z);
    
    //fprintf(f_odom_,"%lf %lf \n", poseOnOdom_.pose.pose.position.x,poseOnOdom_.pose.pose.position.y);  
    // fprintf(f_odom_,"%lf %lf %lf %lf %lf %lf\n", 
    //     current_timeOdom_,
    //     poseOnOdom_.pose.pose.position.x,
    //     poseOnOdom_.pose.pose.position.y,
    //     msg->twist.twist.linear.x,
    //     msg->twist.twist.angular.z,
    //     wLidar_);  

    dtOdom_ = current_timeOdom_ - prev_time;
    vOdom_  = msg->twist.twist.linear.x;
    
    if(dtOdom_ > 0)aOdom_  = (vOdom_- l_vOdom)/dtOdom_;
    else aOdom_ = 0;

    wOdom_  = msg->twist.twist.angular.z;

    fprintf(f_odom_,"%lf %lf %lf %lf %lf %lf\n", 
        current_timeOdom_,
        poseOnOdom_.pose.pose.position.x,
        poseOnOdom_.pose.pose.position.y,
        msg->twist.twist.linear.x,
        wOdom_,
        wLidar_);  

    tf2::Quaternion q(
        poseOnOdom_.pose.pose.orientation.x,
        poseOnOdom_.pose.pose.orientation.y,
        poseOnOdom_.pose.pose.orientation.z,
        poseOnOdom_.pose.pose.orientation.w);
    
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    qOdom_ = yaw;

    // if(prev_time > 0){    
    //     EKF_predict();

    //     fprintf(f_ekf_,"%lf %lf %lf %lf %lf %lf %lf\n",
    //         current_timeOdom_,
    //         dtOdom_,
    //         vOdom_,
    //         wOdom_,
    //         x_est_(0),
    //         x_est_(1), 
    //         x_est_(2));  
    // }
    l_vOdom = vOdom_;
    prev_time = current_timeOdom_;

}

void CloudMatcher::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    tf2::Quaternion q(
        msg->orientation.x,
        msg->orientation.y,
        msg->orientation.z,
        msg->orientation.w);
    
    
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    
    static double pitch_bias = -0.0058;
    static double prev_pitch = 0;
    static double prev_roll = 0;
    
    static double tmp_pitch = 0;    
    static int n = 0;
    static bool flag_cacBias = true;
    double current_timeImu = (double)(msg->header.stamp.sec + RCL_NS_TO_MS(msg->header.stamp.nanosec)*0.001);
    static double init_time = current_timeImu;

    static double prev_time = current_timeImu;
    double dt = current_timeImu - prev_time;

    YawImu_     = -yaw;
    //PitchImu_   =  -pitch;
    PitchImu_   +=  msg->angular_velocity.x*dt;
    //PitchImu_ -= pitch_bias;

//    RollImu_    +=  -(roll - prev_roll);

    //PitchImu_   =  -(pitch);
    //RollImu_    =  -(roll);
    RollImu_    +=  msg->angular_velocity.y*dt;
  
    //printf("imu_callback: pitch_bias=%lf:: row:%lf, pitch:%lf yaw:%lf \n",pitch_bias,roll,pitch,yaw);

    prev_time = current_timeImu;
    prev_pitch = pitch;
    prev_roll = roll;
}

void CloudMatcher::broadcastTF(void) {

        geometry_msgs::msg::TransformStamped t;

        // Read message content and assign it to
        // corresponding tf variables
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id   = "/map";
        t.child_frame_id    = "/base_link";
        
        // Turtle only exists in 2D, thus we get x and y translation
        // coordinates from the message and set the z coordinate to 0
        t.transform.translation.x = poseOnAlign_.pose.pose.position.x;
        t.transform.translation.y = poseOnAlign_.pose.pose.position.y;
        t.transform.translation.z = poseOnAlign_.pose.pose.position.z;

        // For the same reason, turtle can only rotate around one axis
        // and this why we set rotation in x and y to 0 and obtain
        // rotation in z axis from the message
               
        t.transform.rotation.x = poseOnAlign_.pose.pose.orientation.x;
        t.transform.rotation.y = poseOnAlign_.pose.pose.orientation.y;
        t.transform.rotation.z = poseOnAlign_.pose.pose.orientation.z;
        t.transform.rotation.w = poseOnAlign_.pose.pose.orientation.w;

        // Send the transformation
        tf_broadcaster_->sendTransform(t);
}

Eigen::Quaternionf CloudMatcher::QuatMsgToEigen(geometry_msgs::msg::Quaternion q_msg){
    Eigen::Quaternionf q_eigen;

	q_eigen.x() = (float)q_msg.x;
    q_eigen.y() = (float)q_msg.y;
    q_eigen.z() = (float)q_msg.z;
    q_eigen.w() = (float)q_msg.w;
    
    q_eigen.normalize();
	return q_eigen;
}

void CloudMatcher::EKF_init(void){

    prePt_  = Eigen::Matrix3d::Identity(3, 3); //誤差共分散行列
    postPt_ = Eigen::Matrix3d::Identity(3, 3); //誤差共分散行列   
    It_ = Eigen::Matrix3d::Identity(3, 3);      //誤差共分散行列  
    
    Qt_ <<  0.0001, 0.0 , 0.0,
            0.0 , 0.0001, 0.0,
            0.0 , 0.0 , 0.0001; //Covariance Array of System noize   
    
    // Rt_ <<  0.0004,0.0 ,0.0,
    //         0.0 ,0.0004,0.0,
    //         0.0 ,0.0 ,0.0001; //Covariance Array of Sensor noize   

    // Rt_ <<  1.0,0.0 ,
    //         0.0 ,1.0; //Covariance Array of Sensor noize   

    Rt_ <<  0.01,0.0 ,
            0.0 ,0.01; //Covariance Array of Sensor noize   

    //  Ht_ <<  1.0, 0.0, 0.0,
    //          0.0, 1.0, 0.0,
    //          0.0, 0.0, 1.0; //Jacobian   

    Ht_ <<  1.0, 0.0, 0.0,
            0.0, 1.0, 0.0;//Jacobian   

    wNoize_ << 0.01,0.01,0.01; //System Noize 
    vNoize_ << 0.04,0.04,0.09; //Sensor Noize 

    x_est_ << 0,0,0;

    eval_sigma_ = 0;
}

void CloudMatcher::EKF_predict(void){
   
    //double v = vLidar_;
    double v = vOdom_;
    double dt = dtLidar_;
    
    printf("vLidar_=%lf \n",vLidar_);
    x_pre_(2) += wLidar_* dtLidar_; 
    //aOdom_
    double q_tmp = x_pre_(2);//+ wOdom_* dtOdom_;
    //x_pre_(0) += dt*v*cos(q_tmp);  
    //x_pre_(1) += dt*v*sin(q_tmp);  
    double dd = v*dt + 0.5*aOdom_*dt*dt;

    // x_pre_(0) += dt*v*cos(q_tmp);  
    // x_pre_(1) += dt*v*sin(q_tmp);  

    // x_pre_(0) += dd*cos(q_tmp);  
    // x_pre_(1) += dd*sin(q_tmp);  

    x_pre_(0) += dd*cos(q_tmp);  
    x_pre_(1) += dd*sin(q_tmp);  

    // Jacobian
    // Ft_ <<  1.0,0.0,-v*dt*sin(q_tmp),
    //         0.0,1.0,v*dt*cos(q_tmp),
    //         0.0,0.0, 1.0;

    Ft_ <<  1.0,0.0,-dd*sin(q_tmp),
            0.0,1.0,dd*cos(q_tmp),
            0.0,0.0, 1.0;

 
    //予測誤差共分散行列
    prePt_ = Ft_*prePt_*Ft_.transpose() + Qt_;
    
    std::cout << "pre Pt_:"<< std::endl;
    std::cout << prePt_ << std::endl;

    poseErrorCov_.header.stamp.sec = current_timeLidar_;
    poseErrorCov_.header.frame_id = "base_link";
    
    poseErrorCov_.pose.pose.position.x = x_pre_(0);
    poseErrorCov_.pose.pose.position.y = x_pre_(1);
    poseErrorCov_.pose.pose.position.z = Zpose_;
   
    tf2::Quaternion Quat;
    Quat.setRPY( Rollpose_, Pitchpose_, x_pre_(2));
    
    poseErrorCov_.pose.pose.orientation.x = Quat.getX();
    poseErrorCov_.pose.pose.orientation.y = Quat.getY();
    poseErrorCov_.pose.pose.orientation.z = Quat.getZ();
    poseErrorCov_.pose.pose.orientation.w = Quat.getW();
     
    poseErrorCov_.pose.covariance.at(0) = prePt_(0,0);
    poseErrorCov_.pose.covariance.at(7) = prePt_(1,1);
    poseErrorCov_.pose.covariance.at(35) = prePt_(2,2);

    publisher_PoseErrorCov_->publish(std::move(poseErrorCov_));
    
}

void CloudMatcher::EKF_update(void){    

    //x_z_ << Xpose_, Ypose_, Yawpose_;
    //x_z_ << XposeMap_, YposeMap_;
    x_z_ << Xpose_, Ypose_;
    
    x_Zxy_pre_ = x_z_ - Ht_*x_pre_;

    Sxyt_ = Ht_*prePt_*Ht_.transpose()+ Rt_;

    dxy_mh_ = sqrt(x_Zxy_pre_.transpose()*Sxyt_*x_Zxy_pre_);

    if(dxy_mh_ < 0.1){

        //Kalman Gain
        St_ = Ht_*prePt_*Ht_.transpose() + Rt_;
    
        Kt_ = (prePt_*Ht_.transpose())*St_.inverse();
    
        //推定値
        x_est_ = x_pre_ + Kt_*(x_z_ - Ht_*x_pre_);

        //推定誤差共分散行列    
        //postPt_ = prePt_ - Kt_*Ht_*prePt_.transpose();
        postPt_ = ( It_ - Kt_*Ht_)*prePt_;
    
        std::cout << "Kt_:"<< std::endl;
        std::cout << Kt_ << std::endl;

        std::cout << "post Pt_:"<< std::endl;
        std::cout << postPt_ << std::endl;
    
        prePt_ = postPt_;

        double dx = x_pre_(0)-x_est_(0);
        double dy = x_pre_(1)-x_est_(1);
        double gap = sqrt(dx*dx+dy*dy);

        //if(eval_sigma_ < threshold_adjust_pose_){
            //if(gap < 2.0*vLidar_*dtLidar_){ dxy_mh_
            //if(dxy_mh_ > 0 && dxy_mh_ < 0.12){
            x_pre_(0) = x_est_(0);
            x_pre_(1) = x_est_(1);
            x_pre_(2) = x_est_(2);

            // Xpose_= x_pre_(0);
            // Ypose_= x_pre_(1);
            // Yawpose_= x_pre_(2);   
        //}  
    }
 }


int main(int argc, char * argv[])
{
  // クライアントライブラリの初期化
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);
  rclcpp::init(argc, argv);

  // talkerノードの生成とスピン開始
  auto node = std::make_shared<CloudMatcher>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
