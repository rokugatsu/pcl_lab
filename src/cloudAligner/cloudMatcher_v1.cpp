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
    declare_parameter("threshold_align_fitness_score",0.5);
    declare_parameter("save_pcd_dir","");
    declare_parameter("threshold_dist_save_pcd",5.0);
    declare_parameter("Submap_Range",60.0);
    declare_parameter("SumAlign_fitnessScore",500.0);
    declare_parameter("leaf_size",0.5);

    str_pointcloud_map_     = this->get_parameter("pointcloud_map").as_string();
    str_source_pointcloud_  = this->get_parameter("input_pointcloud").as_string();
    str_input_odom_         = this->get_parameter("input_odom").as_string();
    threshold_align_fitness_score_= this->get_parameter("threshold_align_fitness_score").as_double();
    str_save_pcd_dir_       = this->get_parameter("save_pcd_dir").as_string();
    threshold_dist_save_pcd_= this->get_parameter("threshold_dist_save_pcd").as_double();
    SubmapRange_            = this->get_parameter("Submap_Range").as_double();
    SumAlignFitnessScore_   = this->get_parameter("SumAlign_fitnessScore").as_double();
    leafSize_               = this->get_parameter("leaf_size").as_double();

    printf("pointcloud_map=%s \n",str_pointcloud_map_.c_str());
    printf("source_pointcloud=%s \n",str_source_pointcloud_.c_str());
    printf("input_odom=%s \n",str_input_odom_.c_str());
    printf("threshold_align_fitness_score=%lf \n",threshold_align_fitness_score_);
    printf("str_save_pcd_dir_=%s \n",str_save_pcd_dir_.c_str());
    printf("threshold_dist_save_pcd =%lf \n",threshold_dist_save_pcd_);
    printf("Submap_Range =%lf \n",SubmapRange_);
    printf("SumAlign_FitnessScore =%lf \n",SumAlignFitnessScore_);
    
    //poseOnOdom_.header.stamp = 0;
    subscriberLidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      str_source_pointcloud_.c_str(), 10, std::bind(&CloudMatcher::lidar_callback, this, _1));

    subscriberOdom_ = this->create_subscription<nav_msgs::msg::Odometry>(
      str_input_odom_.c_str(), 10, std::bind(&CloudMatcher::odom_callback, this, std::placeholders::_1));

    publisher_SourcePointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/source_pointcloud", 1);
    publisher_TargetPointcloud_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/target_pointcloud", 1);
    publisher_AlignPointcloud_  = this->create_publisher<sensor_msgs::msg::PointCloud2>("/aligned_pointcloud", 10);
    publisher_PointcloudMap_    = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_pointcloud", 10);
    publisher_PointcloudMapAlign_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_aligned_pointcloud", 10);
    publisher_PointcloudMapAlignSum_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/sum_aligned_pointcloud", 10);
    publisher_PoseVis_  = this->create_publisher<visualization_msgs::msg::Marker>("pose_track", 10);

    publisher_PointcloudMapMatching_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/mapmatching_pointcloud", 1);

    timer_ = create_wall_timer(100ms, publish_AlignMap);

    // Initialize the transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    load_pointcloud(str_pointcloud_map_.c_str());

    count_RosbagPlay_ = -100;

    dx_ = 0;
    dy_ = 0;
    dyaw_ = 0;
    
    Xpose_ = 0;
    Ypose_ = 0;
    Yawpose_ = 0;

    std::string str_pose_data = str_save_pcd_dir_;
    str_pose_data += "pose.dat";

    std::string str_odom_data = str_save_pcd_dir_;
    str_odom_data += "odom.dat";

    f_pose_ = fopen(str_pose_data.c_str(),"w");
    f_odom_ = fopen( str_odom_data.c_str(),"w");

    std::cout << "--- pcl::GICP ---" << std::endl;
    boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>> gicp(new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());
    ptr_gicp_ = gicp;   

    std::cout << "--- pcl::NDTOMP ---" << std::endl;
    //pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>::Ptr ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>()); //ndt_omp->setResolution(5.0);
 
    boost::shared_ptr<pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>> ndt_omp(new pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>());
    ndt_omp->setNumThreads(omp_get_max_threads());
    ndt_omp->setNeighborhoodSearchMethod(pclomp::DIRECT7);

    ptr_ndt_omp_ = ndt_omp;   


    //-- visualization on rviz
    poseVis_.id = 0;
    poseVis_.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    poseVis_.scale.x = poseVis_.scale.y = 1.0;
    poseVis_.color.r = 0.85f;
    poseVis_.color.g = 0.0f;
    poseVis_.color.b = 0.0f;
    poseVis_.color.a = 1.0; 
    poseVis_.points.clear();

    poseVis_.header.stamp = this->now();

    poseVis_.pose.position.x = 0;
    poseVis_.pose.position.y = 0;
    poseVis_.pose.position.z = 0;

    poseVis_.pose.orientation.x = 0;
    poseVis_.pose.orientation.y = 0;
    poseVis_.pose.orientation.z = 0;
    poseVis_.pose.orientation.w = 0;
    
    poseVis_.header.frame_id ="base_link";
    poseVis_.action = visualization_msgs::msg::Marker::ADD;

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
    
    dyaw_ = yaw;
    dpitch_ = pitch;
    droll_ = roll;

    double dd = sqrt(dx_*dx_ + dy_*dy_ + dz_*dz_);
    
    Yawpose_    += -yaw;
    Pitchpose_  = -pitch;
    Rollpose_   = -roll;
    
    Xpose_ += dd*cos(Pitchpose_)*cos(Yawpose_);
    Ypose_ += dd*cos(Pitchpose_)*sin(Yawpose_);
    Zpose_ = dz_;

    printf("align:: dd =%lf x:%lf, y:%lf, z:%lf, roll:%lf, pitch:%lf, yaw:%lf\n",
        dd,
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
    
    dyaw_ = yaw;
    dpitch_ = pitch;
    droll_ = roll;

    //double x_current = Xpose_; 
    //double y_current = Ypose_; 
    double x_current = dx_; 
    double y_current = dy_; 

    Xpose_ += cos(dyaw_)*dx_ - sin(dyaw_)*dy_;
    //Xpose_ += dx_;

    Ypose_ += sin(dyaw_)*dx_ + cos(dyaw_)*dy_;
    //Ypose_ += dy_;

    Yawpose_    += yaw;
    // Pitchpose_  = -pitch;
    // Rollpose_   = -roll;


    double dd = sqrt(dx_*dx_ + dy_*dy_ + dz_*dz_);
/*    
    Yawpose_    += -yaw;
    Pitchpose_  = -pitch;
    Rollpose_   = -roll;
    
    Xpose_ += dd*cos(Pitchpose_)*cos(Yawpose_);
    Ypose_ += dd*cos(Pitchpose_)*sin(Yawpose_);
    Zpose_ = dz_;
*/
    printf("align:: dd =%lf x:%lf, y:%lf, z:%lf, roll:%lf, pitch:%lf, yaw:%lf\n",
        dd,
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


void CloudMatcher::lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg)
{
    static unsigned int n=0 ;
    static int num=0;
    //static int n_submap = 0;
    static char strc_pcd[32];
    static std::string str_file;

    pcl::fromROSMsg(*cloud_msg, PointcloudRaw_pcl_); 
    PointcloudRaw_pcl_.header.frame_id = "lidar";
    
    // Lidar Odometry
    if(PointcloudTarget_pcl_.size () > 0 && PointcloudTarget_ros_.data.size () > 0){
        PointcloudSource_pcl_ = PointcloudTarget_pcl_;
        PointcloudSource_ros_ = PointcloudTarget_ros_;
    }

    pcl::VoxelGrid<pcl::PointXYZ> voxelSampler;
    voxelSampler.setInputCloud(PointcloudRaw_pcl_.makeShared());
    voxelSampler.setLeafSize(leafSize_, leafSize_ , leafSize_ );
    voxelSampler.setFilterFieldName("z");  // Z軸（高さ）の値でフィルタをかける
    voxelSampler.setFilterLimits(0.1, 50.0);  // 0.1 ～ 50.0 m の間にある点群を抽出
    voxelSampler.filter(PointcloudTarget_pcl_);

    pcl::toROSMsg(PointcloudTarget_pcl_, PointcloudTarget_ros_); 

    publisher_SourcePointcloud_ ->publish(std::move(PointcloudSource_ros_));
    publisher_TargetPointcloud_ ->publish(std::move(PointcloudTarget_ros_));

    if(PointcloudTarget_pcl_.size () > 0 && PointcloudSource_pcl_.size () > 0){

        //Step1: Lidar Odometry    
        printf("--- Step1: Lidar Odometry -------------- \n");
        pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_lidarOdom = align(ptr_gicp_, PointcloudTarget_pcl_.makeShared(),  PointcloudSource_pcl_.makeShared());
        convertTransformEigenToROS();
        Eigen::Affine3f transformatoin = pcl::getTransformation(Xpose_, Ypose_, Zpose_, Rollpose_, Pitchpose_, Yawpose_);
	    pcl::transformPointCloud(*aligned_lidarOdom, PointcloudAlign_pcl_, transformatoin);
        pcl::toROSMsg(PointcloudAlign_pcl_, PointcloudAlign_ros_); 

        fprintf(f_pose_,"%d %lf %lf %lf %lf %lf %lf %lf\n",
            n++,Xpose_,Ypose_, Zpose_,Yawpose_*180/M_PI,Pitchpose_*180/M_PI,Rollpose_*180/M_PI,align_fitnessScore_);  

        if(n == 0 || (n > 20 && n %20 == 0)){
            geometry_msgs::msg::Point point_geometry;
            point_geometry.x = -Xpose_;
            point_geometry.y = -Ypose_;
            point_geometry.z = Zpose_;

            poseVis_.header.stamp = this->now();
            poseVis_.points.push_back(point_geometry);
        }

        
        //Step2: Scan matching with map    
        if(num == 0 || align_fitnessScore_ < threshold_align_fitness_score_ ){
            
            printf("--- Step2: Scan matching with map : %lf / %lf -------------- \n", align_fitnessScore_, threshold_align_fitness_score_);
            
            pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_mapmatching = align(ptr_ndt_omp_, PointcloudMap_pcl_.makeShared(), PointcloudAlign_pcl_.makeShared());
            //pcl::PointCloud<pcl::PointXYZ>::Ptr aligned_mapmatching = align(ptr_gicp_, PointcloudMap_pcl_.makeShared(), PointcloudAlign_pcl_.makeShared());
            
            if(align_fitnessScore_ < 0.15){
                //convertTransformEigenToROS();
                convertTransformEigenToROS_MapMatching();
                //Eigen::Affine3f transformatoin = pcl::getTransformation(-dx_, -dy_, -dz_, -droll_, -dpitch_, -dyaw_);	        
                //Eigen::Affine3f transformatoin = pcl::getTransformation(dx_, dy_, dz_, droll_, dpitch_, dyaw_);	        
            
                //pcl::transformPointCloud(PointcloudAlign_pcl_, PointcloudAlign_pcl_, transformatoin);
                //pcl::toROSMsg(PointcloudAlign_pcl_, PointcloudAlign_ros_);     
            
                pcl::toROSMsg(*aligned_mapmatching, PointcloudMapMatching_ros_); 
                publisher_PointcloudMapMatching_->publish(std::move(PointcloudMapMatching_ros_));
            }
        }
            
        
        pcl::toROSMsg(PointcloudAlign_pcl_, PointcloudAlign_ros_);  
        publisher_AlignPointcloud_ ->publish(std::move(PointcloudAlign_ros_));
        publisher_PoseVis_->publish(std::move(poseVis_));
            
        broadcastTF();
    }
        
    count_RosbagPlay_ = 0;
}

void CloudMatcher::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    prev_poseOnOdom_ = poseOnOdom_;
    
    poseOnOdom_ = *msg;
    poseOnOdom_.header.frame_id = "base_link";
    
    fprintf(f_odom_,"%lf %lf \n", poseOnOdom_.pose.pose.position.x,poseOnOdom_.pose.pose.position.y);  
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
