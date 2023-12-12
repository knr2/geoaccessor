#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/io.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/point_types.h>
#include <pcl/filters/approximate_voxel_grid.h>
// opencv
#include <boost/bind.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <tf/transform_broadcaster.h>
// settin parm
const double PI = acos(-1.0);
//
double area_size = 0., cheak_size;
int goal_point_number = 0;
std::string audio_path = "", goal_path = "";
int search_time = 0;
pcl::PointCloud<pcl::PointXYZ>::Ptr goal_clouds(new pcl::PointCloud<pcl::PointXYZ>);
int count = 0;
bool call_flag = true;
double distance = 0.;
std::string score;

void launchExternalApplication() {
    // 外部アプリケーションを起動するためのコマンドを定義
    std::string command = audio_path + score + std::string(".mp3");

    try {
        // コマンドを実行
        int result = std::system(command.c_str());

        if (result == 0) {
            ROS_INFO("External application launched successfully.");
        } else {
            ROS_ERROR("Error launching external application. Return code: %d", result);
        }
    } catch (const std::exception& e) {
        ROS_ERROR("Exception occurred while launching external application: %s", e.what());
    }
}

void callback(const sensor_msgs::PointCloud::ConstPtr& point)
{
  count++;
  ROS_INFO("[%d]:Get Date Now!!", count);
  if(count >= search_time * 10 && call_flag)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr load_clouds(new pcl::PointCloud<pcl::PointXYZ>), cheak_area(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 point2;
    convertPointCloudToPointCloud2(*point, point2);
    pcl::fromROSMsg(point2, *load_clouds);
    for(const auto& point:*load_clouds)
    {
      if(sqrt(pow(point.x, 2) + pow(point.y, 2)) - 0.25 <= cheak_size)
      {
        pcl::PointXYZ pt;
        pt.x = point.x;
        pt.y = point.y;
        pt.z = point.z;
        cheak_area->push_back(pt);
      }
    }
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setMaximumIterations(10000);
    icp.setTransformationEpsilon(1e-9);
    icp.setEuclideanFitnessEpsilon(0.005);
    icp.setMaxCorrespondenceDistance(0.05);
    icp.setInputSource(load_clouds);
    icp.setInputTarget(goal_clouds);
    icp.align(*load_clouds);
    Eigen::Matrix4f transformation = icp.getFinalTransformation();
    distance = sqrt(pow(transformation(0, 3), 2) + pow(transformation(1, 3), 2));
    score = "";
    if(distance/area_size <= 0.03)
      score = "S++";
    else if(distance/area_size <= 0.05)
      score = "S+";
    else if(distance/area_size <= 0.1)
      score = "S";
    else if(distance/area_size <= 0.2)
      score = "A";
    else if(distance/area_size <= 0.6)
      score = "B";
    else if(distance/area_size <= 0.8)
      score = "C";
    else
      score = "D";
    ROS_INFO("%s", score.c_str());
    ros::shutdown();
  }
}
int main(int argc, char **argv)
{
  std::cout << "test";
  // set parameters
  ros::init(argc, argv, "Geoaccessor");
  ros::param::get("/geoaccessor/area_size", area_size);
  ros::param::get("/geoaccessor/cheak_size", cheak_size);
  ros::param::get("/geoaccessor/goal_point_number", goal_point_number);
  ros::param::get("/geoaccessor/goal_path", goal_path);
  ros::param::get("/geoaccessor/audio_path", audio_path);
  ros::param::get("/geoaccessor/search_time", search_time);
  ros::NodeHandle ydlidar;
  std::string goal_name = goal_path + std::to_string(goal_point_number) + std::string(".pcd");
  pcl::io::loadPCDFile(goal_name, *goal_clouds);
  ros::Subscriber Geoaccessor = ydlidar.subscribe("/point_cloud", 1000, callback);
  while(ros::ok()) 
  {
    ros::spin();
  }
  launchExternalApplication();
  return 0;
}
