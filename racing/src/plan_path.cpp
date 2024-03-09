/// \file plan_path.cpp
/// \brief Commands the car to drive in a circle
///
/// PARAMETERS:
///     rate (int): the publishing frequency (Hz)
///     angular_velocity (double): the angular velocity of the robot (m)
/// PUBLISHES:
///     cmd_vel (geometry_msgs::msg::Twist): velocity commands for the robot
/// CLIENTS:
///     enable_drive (std_srvs::srv::SetBool): enables/disbales drive

#include <chrono>
#include <vector>
#include <cmath>
#include <string>
#include <yaml-cpp/yaml.h>
#include <fstream>
#include <nav_msgs/msg/path.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

class Plan_Path : public rclcpp::Node
{
public:
  Plan_Path()
  : Node("plan_path")
  {
    // Parameters and default values
    declare_parameter("rate", 100.);
    declare_parameter("min_spacing", 1.);
    declare_parameter("filename", "path.yaml");

    // Define parameter variables
    loop_rate = get_parameter("rate").as_double();
    min_spacing = get_parameter("min_spacing").as_double();
    filename = get_parameter("filename").as_string();

    // Other variables

    // Update ground truth red turtle path
    planned_path.header.stamp = get_clock()->now();
    planned_path.header.frame_id = "map";

    // Create current pose stamped
    current_pose.header.stamp = get_clock()->now();
    current_pose.header.frame_id = "map";
    current_pose.pose.position.x = 0.0;
    current_pose.pose.position.y = 0.0;
    current_pose.pose.position.z = 0.0;
    current_pose.pose.orientation.x = 0.0;
    current_pose.pose.orientation.y = 0.0;
    current_pose.pose.orientation.z = 0.0;
    current_pose.pose.orientation.w = 1.0;

    // Add current pose to path
    planned_path.poses.push_back(current_pose);

    // Create prev pose stamped
    prev_pose.header.stamp = get_clock()->now();
    prev_pose.header.frame_id = "map";
    prev_pose.pose.position.x = 0.0;
    prev_pose.pose.position.y = 0.0;
    prev_pose.pose.position.z = 0.0;
    prev_pose.pose.orientation.x = 0.0;
    prev_pose.pose.orientation.y = 0.0;
    prev_pose.pose.orientation.z = 0.0;
    prev_pose.pose.orientation.w = 1.0;
    
    // Create planned path and planned_path
    planned_path.header.stamp = get_clock()->now();
    planned_path.header.frame_id = "map";

    // Publishers
    planned_path_pub = create_publisher<nav_msgs::msg::Path>("path_plan", 10);

    // Servers
    save_path_srv = create_service<std_srvs::srv::Empty>(
      "save_path",
      std::bind(&Plan_Path::save_path_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Main timer
    int cycle_time = 1000.0 / loop_rate;
    planned_path_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Plan_Path::publish_planned_path, this));
      
    // Transform broadcaster and listener
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  }

private:
  // Initialize parameter variables
  int rate;
  double loop_rate, min_spacing;
  geometry_msgs::msg::PoseStamped current_pose, prev_pose;
  nav_msgs::msg::Path planned_path;
  std::string filename;

  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_path_srv;
  rclcpp::TimerBase::SharedPtr planned_path_timer;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;

  /// \brief Updates the robot path with the current pose and publishes the path
  void publish_planned_path()
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      // Update ground truth red turtle path
      planned_path.header.stamp = get_clock()->now();
      planned_path.header.frame_id = "map";

      // Add current pose to path
      transformStamped = tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);

      // Create new pose stamped
      current_pose.header.stamp = get_clock()->now();
      current_pose.header.frame_id = "map";
      current_pose.pose.position.x = transformStamped.transform.translation.x;
      current_pose.pose.position.y = transformStamped.transform.translation.y;
      current_pose.pose.position.z = 0.0;

      // Add rotation quaternion about Z
      current_pose.pose.orientation.x = transformStamped.transform.rotation.x;
      current_pose.pose.orientation.y = transformStamped.transform.rotation.y;
      current_pose.pose.orientation.z = transformStamped.transform.rotation.z;
      current_pose.pose.orientation.w = transformStamped.transform.rotation.w;

      double dist = sqrt(pow((current_pose.pose.position.x - prev_pose.pose.position.x), 2) + pow((current_pose.pose.position.y - prev_pose.pose.position.y), 2));

      if (dist > min_spacing) {
        // Add pose to path 
        planned_path.poses.push_back(current_pose);
        // Save current pose as previous pose
        prev_pose = current_pose;
      }
      
      // Publish path
      planned_path_pub->publish(planned_path);

    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not transform from map to base_link: %s", ex.what());
    }
  }

  /// \brief Callback for starting the race
  void save_path_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    // Save path to YAML
    savePathToYamlFile();

    // Reset planned path
    nav_msgs::msg::Path empty_path;
    planned_path = empty_path;
  }

  /// \brief Convert geometry_msgs::msg::PoseStamped to YAML Node. Citation (1): ChatGPT
  YAML::Node poseToYaml(const geometry_msgs::msg::PoseStamped& pose) {
      YAML::Node node;
      node["position"]["x"] = pose.pose.position.x;
      node["position"]["y"] = pose.pose.position.y;
      node["position"]["z"] = pose.pose.position.z;
      node["orientation"]["x"] = pose.pose.orientation.x;
      node["orientation"]["y"] = pose.pose.orientation.y;
      node["orientation"]["z"] = pose.pose.orientation.z;
      node["orientation"]["w"] = pose.pose.orientation.w;
      return node;
  }

  /// \brief Convert nav_msgs::msg::Path to YAML Node. Citation (1): ChatGPT
  YAML::Node pathToYaml(const nav_msgs::msg::Path& path) {
      YAML::Node node;
      for (const auto& pose : path.poses) {
          node["poses"].push_back(poseToYaml(pose));
      }
      return node;
  }

  /// \brief Serialize and save the path to a YAML file. Citation (1): ChatGPT
  void savePathToYamlFile() {
      std::ofstream fout(filename);
      if (fout.is_open()) {
          YAML::Node yamlPath = pathToYaml(planned_path);
          fout << yamlPath;
      }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Plan_Path>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
