#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/search/flann_search.h>
#include <pcl/io/pcd_io.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

//#include "cloudLab.hpp"

#ifndef CLOUD_CREATOR_HPP_
#define CLOUD_CREATOR_HPP_


typedef struct pointcloudSet {
  pcl::PointCloud<pcl::PointXYZ> pointcloud;
  std::string file_path;
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;      
}pointcloudSet;


class CloudAligner : public rclcpp::Node
{
  public:
    CloudAligner();
    void load_pointcloud(const std::string &file_name);
    void create_pointcloud(void);
    void pointcloud_Base(const std::string &file_name);
    void pointcloud_Move(const std::string &file_name);
    void align_to_target(void);

    Eigen::Quaternionf QuatMsgToEigen(geometry_msgs::msg::Quaternion q_msg);

    void convertTransformEigenToROS(void);
    bool Is_LoadPCDFile(void);
   
    void timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void broadcastTF(void);

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_SourcePointcloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_TargetPointcloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_AlignPointcloud_;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_PointcloudMap_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_PointcloudMapAlign_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_PointcloudMapAlignSum_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_[3];
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_PoseVis_;
    
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriberLidar_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriberOdom_;

    size_t count_;

    std::string file_path_[2];
    pcl::PointCloud<pcl::PointXYZ> PointcloudMap_pcl_;
    pcl::PointCloud<pcl::PointXYZ> PointcloudRaw_pcl_;
    pcl::PointCloud<pcl::PointXYZ> PointcloudSource_pcl_;
    pcl::PointCloud<pcl::PointXYZ> PointcloudTarget_pcl_;
    pcl::PointCloud<pcl::PointXYZ> PointcloudAlign_pcl_;
    pcl::PointCloud<pcl::PointXYZ> PointcloudMapAlign_pcl_;
    
    pcl::PointCloud<pcl::PointXYZ> PointcloudMapAlignSum_pcl_;
    pcl::PointCloud<pcl::PointXYZ> cloud_pcl_[3];

    sensor_msgs::msg::PointCloud2 PointcloudTarget_ros_;
    sensor_msgs::msg::PointCloud2 PointcloudSource_ros_;
    sensor_msgs::msg::PointCloud2 PointcloudAlign_ros_;
    sensor_msgs::msg::PointCloud2 PointcloudMap_ros_;
    sensor_msgs::msg::PointCloud2 PointcloudMapAlign_ros_;
    sensor_msgs::msg::PointCloud2 PointcloudMapAlignSum_ros_;

    std::string str_pointcloud_map_;
    std::string str_source_pointcloud_;  
    std::string str_target_pointcloud_;
    std::string str_input_odom_; 
    std::string str_save_pcd_dir_;

    sensor_msgs::msg::PointCloud2 cloud_ros_[3];
    geometry_msgs::msg::Pose transform_ros_;
    
    Eigen::Matrix4d transform_eigen_;

    geometry_msgs::msg::Pose poseOnMap_;
    nav_msgs::msg::Odometry poseOnOdom_;
    nav_msgs::msg::Odometry prev_poseOnOdom_;
    nav_msgs::msg::Odometry poseOnAlign_;
   
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    tf2_ros::TransformListener* tf2_listener_;
    
    boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>> ptr_gicp_;
    boost::shared_ptr<pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>> ptr_ndt_;
 
    pcl::PointCloud<pcl::PointXYZ>::Ptr align(boost::shared_ptr<pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>> registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud );
    pcl::PointCloud<pcl::PointXYZ>::Ptr align_wo_init_trans(boost::shared_ptr<pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>> registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud );
 
    std::FILE* f_pose_;
    std::FILE* f_odom_;

    bool is_loadPCDFile_;
    double leafSize_;

    double dx_;
    double dy_;
    double dz_;
    
    double dyaw_;
    
    double Xpose_;
    double Ypose_;
    double Zpose_;
    
    double Yawpose_;
    double Pitchpose_;
    double Rollpose_;
    
    double XposeOrg_;
    double YposeOrg_;
    double ZposeOrg_;
    
    double DistSubmap_;

    double align_fitnessScore_;
    double threshold_align_fitness_score_;
    double threshold_dist_save_pcd_;
    double SubmapRange_;
    double SumAlignFitnessScore_;

    std::vector <pointcloudSet> vec_tmp_pc_set_;
    std::vector <std::vector<pointcloudSet>> vec_pc_set_;
    pointcloudSet tmp_pc_set_;

    std::vector <pointcloudSet> vec_submapA_;
    pointcloudSet tmp_submap_;

    int count_RosbagPlay_;

    visualization_msgs::msg::Marker poseVis_;
};

#endif
