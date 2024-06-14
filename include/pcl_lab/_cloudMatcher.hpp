#include <chrono>
#include <cstdio>
#include <memory>
#include <string>
#include <vector>
#include <deque>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>

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

// ----- NDT OMP-----
#include <pclomp/ndt_omp.h>
//#include <pclomp/gicp_omp.h>
// #include <multigrid_pclomp/multi_voxel_grid_covariance_omp_impl.hpp>
// #include <multigrid_pclomp/multi_voxel_grid_covariance_omp.h>
// #include <multigrid_pclomp/multigrid_ndt_omp_impl.hpp>
// #include <multigrid_pclomp/multigrid_ndt_omp.h>
// ----------------

#include <tf2/transform_datatypes.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/transform_broadcaster.h"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <tf2_eigen/tf2_eigen.hpp>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>  // 固有値・固有ベクトル計算 etc. （外積）

#ifndef CLOUD_CREATOR_HPP_
#define CLOUD_CREATOR_HPP_

#define LOG
using namespace Eigen;

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

typedef struct point2D {
  double x;
  double y;
}point2D;

typedef struct point3D {
  double x;
  double y;
  double z;
}point3D;

class CloudMatcher : public rclcpp::Node
{
  public:
    
    inline double modAngle(double a) {    
        while (a < -M_PI)
            a += 2.0 * M_PI;
        while (a > M_PI)
            a -= 2.0 * M_PI;
        
        return a;
    }

    CloudMatcher();
    
    void load_pointcloud(const std::string &file_name);
    void create_pointcloud(void);
    void pointcloud_Base(const std::string &file_name);
    void pointcloud_Move(const std::string &file_name);
    void align_to_target(void);

    Eigen::Quaternionf QuatMsgToEigen(geometry_msgs::msg::Quaternion q_msg);

    void convertTransformEigenToROS(void);
    void convertTransformEigenToROS_MapMatching(void);
    void accept_MapMatching(void);

    bool Is_LoadPCDFile(void);
   
    void timer_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr cloud_msg);
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void broadcastTF(void);

    //EKF
    void EKF_init(void);
    void EKF_predict(void);
    void EKF_update(void);
    int  EKF_evalSensing(void);

  private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_SourcePointcloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_TargetPointcloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_AlignPointcloud_;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_PointcloudMap_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_PointcloudMapAlign_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_PointcloudMapAlignSum_;
    
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_PointcloudMapMatching_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_[3];
    
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_PredictPoseVis_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_EstimatePoseVis_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_MapPoseVis_;

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_PoseErrorCov_;
    
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscriberLidar_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriberOdom_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriberImu_;


    size_t count_;

    std::string file_path_[2];
    pcl::PointCloud<pcl::PointXYZ> PointcloudMap_pcl_;
    pcl::PointCloud<pcl::PointXYZ> PointcloudRaw_pcl_;
    pcl::PointCloud<pcl::PointXYZ> PointcloudSource_pcl_;
    pcl::PointCloud<pcl::PointXYZ> PointcloudTarget_pcl_;

    pcl::PointCloud<pcl::PointXYZ> PointcloudAlign_pcl_;
    pcl::PointCloud<pcl::PointXYZ> PointcloudAlign2_pcl_;
    
    //pcl::PointCloud<pcl::PointXYZ> PointcloudAlign1_pcl_;
    //pcl::PointCloud<pcl::PointXYZ> PointcloudAlign2_pcl_;
    
    pcl::PointCloud<pcl::PointXYZ> PointcloudAlignVoxel_pcl_;
    
    pcl::PointCloud<pcl::PointXYZ> PointcloudMapAlign_pcl_;
    pcl::PointCloud<pcl::PointXYZ> PointcloudMapMatching_pcl_;


    pcl::PointCloud<pcl::PointXYZ> PointcloudMapAlignSum_pcl_;
    pcl::PointCloud<pcl::PointXYZ> cloud_pcl_[3];

    sensor_msgs::msg::PointCloud2 PointcloudTarget_ros_;
    sensor_msgs::msg::PointCloud2 PointcloudSource_ros_;
    sensor_msgs::msg::PointCloud2 PointcloudAlign_ros_;
    sensor_msgs::msg::PointCloud2 PointcloudAlign1_ros_;
    sensor_msgs::msg::PointCloud2 PointcloudAlign2_ros_;


    sensor_msgs::msg::PointCloud2 PointcloudAlignVoxel_ros_;
    
    sensor_msgs::msg::PointCloud2 PointcloudMap_ros_;
    sensor_msgs::msg::PointCloud2 PointcloudMapAlign_ros_;
    
    sensor_msgs::msg::PointCloud2 PointcloudMapAlignSum_ros_;
    sensor_msgs::msg::PointCloud2 PointcloudMapMatching_ros_;

    std::string str_pointcloud_map_;
    std::string str_source_pointcloud_;  
    std::string str_target_pointcloud_;
    std::string str_input_odom_; 
    std::string str_input_imu_;
    std::string str_save_pcd_dir_;
    

    sensor_msgs::msg::PointCloud2 cloud_ros_[3];
    geometry_msgs::msg::Pose transform_ros_;
    geometry_msgs::msg::Point predict_point_geometry_;
    
    Eigen::Matrix4d transform_eigen_;

    geometry_msgs::msg::PoseWithCovarianceStamped poseErrorCov_;

    nav_msgs::msg::Odometry poseOnOdom_;
    nav_msgs::msg::Odometry prev_poseOnOdom_;
    nav_msgs::msg::Odometry poseOnAlign_;
   
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    tf2_ros::TransformListener* tf2_listener_;
    
    boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>> ptr_gicp_;
    boost::shared_ptr<pcl::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>> ptr_ndt_;
    boost::shared_ptr<pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>> ptr_ndt_omp_;
  
    pcl::PointCloud<pcl::PointXYZ>::Ptr align(boost::shared_ptr<pcl::Registration<pcl::PointXYZ, pcl::PointXYZ>> registration, const pcl::PointCloud<pcl::PointXYZ>::Ptr& target_cloud, const pcl::PointCloud<pcl::PointXYZ>::Ptr& source_cloud );
    
    std::FILE* f_pose_;
    std::FILE* f_odom_;
    std::FILE* f_ekf_;

    bool is_loadPCDFile_;
    double leafSize_;
    double matching_leafSize_;

    double current_timeOdom_;
    double current_timeLidar_;
    
    double dd_;
    double dx_;
    double dy_;
    double dz_;
  
    double dyaw_;
    double dpitch_;
    double droll_;

    double dgap_;

    double ddMap_;
    double dxMap_;
    double dyMap_;
    double dzMap_;

    double dyawMap_;
    double dpitchMap_;
    double drollMap_;
    
    double Xpose_;
    double Ypose_;
    double Zpose_;
    
    double Yawpose_;
    double Pitchpose_;
    double Rollpose_;

    double tmp_Xpose_;
    double tmp_Ypose_;
    double tmp_Zpose_;

    double tmp_Yawpose_;
    double tmp_Pitchpose_;
    double tmp_Rollpose_;

    int n_EKF_eval_;

    //GICP
    double  GICP_TransformationEpsilon_;
    double  GICP_EuclideanFitnessEpsilon_;
    int     GICP_MaximumIterations_; 

    //NDT_OMP
    double NDT_OMP_TransformationEpsilon_;//(0.1);
    double NDT_OMP_StepSize_;//(0.75); //<- これかも
    double NDT_OMP_Resolution_;//(5.0);
    int NDT_OMP_MaximumIterations_;//(30);


    double align_fitnessScore_;
    double threshold_score_align_with_map_ ;
    double threshold_adjust_pose_ ;
    
    // Lidar odom
    double threshold_align_fitness_score_;
    double threshold_dist_save_pcd_;
    double SubmapRange_;
    double SumAlignFitnessScore_;

    //EKF
    double threshold_EKF_align_fitness_score_;
    double threshold_mh_dist_;

    std::vector <pointcloudSet> vec_tmp_pc_set_;
    std::vector <std::vector<pointcloudSet>> vec_pc_set_;
    pointcloudSet tmp_pc_set_;

    std::vector <pointcloudSet> vec_submapA_;
    pointcloudSet tmp_submap_;

    int count_RosbagPlay_;
    
    visualization_msgs::msg::Marker PrePoseVis_;    
    visualization_msgs::msg::Marker EstPoseVis_;  
    visualization_msgs::msg::Marker MapPoseVis_;

    //For EKF
    Eigen::Matrix3d Ft_;
    Eigen::Matrix<double, 2, 3> Ht_;
 
    Eigen::Matrix3d prePt_;
    Eigen::Matrix3d postPt_;
    
    Eigen::Matrix2d St_;
    Eigen::Matrix3d Sxyzt_;

    Eigen::Matrix2d Rt_;
    Eigen::Matrix3d Qt_;
    Eigen::Matrix<double, 3, 2> Kt_;
    Eigen::Matrix3d It_;

    Eigen::Vector3d wNoize_;
    Eigen::Vector3d vNoize_;

    double dtOdom_;
    double dtLidar_;

    double vOdom_;
    double aOdom_;
    double wOdom_;
    double qOdom_;

    double vLidar_;    
    double wLidar_;

    double yawAdjust_;
  
    Eigen::Vector3d x_pre_;
    Eigen::Vector3d x_est_;
    Eigen::Vector2d x_z_;

    double dxy_mh_;
    
    double Sx2_;
    double Sy2_;
    double Sz2_;
    
    double YawImu_;
    double PitchImu_;
    double RollImu_;

    bool flag_AlignedWithMap_;

    std::deque <point3D> dq_Xpose_;
};

#endif
