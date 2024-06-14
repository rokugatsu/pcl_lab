#include <chrono>
#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pcl_lab/cloudAligner.hpp>


using namespace std::chrono_literals;
using std::placeholders::_1;