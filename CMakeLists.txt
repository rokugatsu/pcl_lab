cmake_minimum_required(VERSION 3.14)
project(pcl_lab)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE "RELEASE")

include(GNUInstallDirs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(PCL 1.12 REQUIRED COMPONENTS common io registration)
find_package(pcl_conversions REQUIRED PATHs /opt/ros/humble)

find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(Boost REQUIRED system)

find_package(pcl_ros)
find_package(geometry_msgs)
find_package(nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(eigen3_cmake_module REQUIRED)

#find_package(OpenMP REQUIRED)


include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  "/opt/ros/humble/include/tf2_eigen/"
  "/opt/ros/humble/include/tf2/"
  "/opt/ros/humble/include/tf2_ros/"
  "/opt/ros/humble/include/tf2_msgs/"
  "/opt/ros/humble/include/tf2_geometry_msgs/"
  "/opt/ros/humble/include/tf2_eigen/tf2_eigen/"
  "/opt/ros/humble/include/nav_msgs/"
  "/opt/ros/humble/include/visualization_msgs/"
  "/home/june/colcon_ws/src/ndt_omp_ros2/include/"
)

link_directories(
  ${PCL_LIBRARY_DIRS}
  ${Boost_LIBRARY_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_library(ndt_omp
  /home/june/colcon_ws/src/ndt_omp_ros2/src/pclomp/gicp_omp.cpp
  /home/june/colcon_ws/src/ndt_omp_ros2/src/pclomp/ndt_omp.cpp
  /home/june/colcon_ws/src/ndt_omp_ros2/src/pclomp/voxel_grid_covariance_omp.cpp
)

#############################
## ROS 2 multigrid ndt_omp ##
#############################
#ament_auto_add_library(multigrid_ndt_omp SHARED
#  src/multigrid_pclomp/multi_voxel_grid_covariance_omp.cpp
#  src/multigrid_pclomp/multigrid_ndt_omp.cpp
#)
#target_link_libraries(multigrid_ndt_omp ${PCL_LIBRARIES})
#if(OpenMP_CXX_FOUND)
#  target_link_libraries(multigrid_ndt_omp OpenMP::OpenMP_CXX)
#else()
#  message(WARNING "OpenMP not found")
#endif()


add_library(multigrid_ndt_omp
  /home/june/colcon_ws/src/ndt_omp_ros2/src/multigrid_pclomp/multi_voxel_grid_covariance_omp.cpp
  /home/june/colcon_ws/src/ndt_omp_ros2/src/multigrid_pclomp/multigrid_ndt_omp.cpp
)

add_executable(cloudAligner src/cloudAligner/cloudAligner.cpp )
ament_target_dependencies(cloudAligner rclcpp geometry_msgs sensor_msgs nav_msgs pcl_conversions tf2_ros visualization_msgs)
target_link_libraries(cloudAligner ${PCL_LIBRARIES})

add_executable(cloudMatcher src/cloudMatcher/cloudMatcher.cpp )
add_dependencies(cloudMatcher ndt_omp multigrid_ndt_omp)
ament_target_dependencies(cloudMatcher rclcpp geometry_msgs sensor_msgs nav_msgs pcl_conversions tf2_ros visualization_msgs)
target_link_libraries(cloudMatcher ${PCL_LIBRARIES} ndt_omp multigrid_ndt_omp)


install(TARGETS
    cloudAligner
    cloudMatcher
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# install headers
install(
  DIRECTORY "include/"
  DESTINATION include
)

ament_export_dependencies(rclcpp)
ament_export_include_directories(include)

ament_package()
