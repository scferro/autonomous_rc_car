/// \file race_path.cpp
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
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

class Race_Path : public rclcpp::Node
{
public:
  Race_Path()
  : Node("race_path")
  {
    // Parameters and default values
    declare_parameter("rate", 50.);
    declare_parameter("cmd_max", 2000);
    declare_parameter("cmd_min", 1000);
    declare_parameter("max_rpm", 16095.);
    declare_parameter("wheel_diameter", 0.108);
    declare_parameter("gear_ratio", 5.);
    declare_parameter("Kp_steer", 8.0);
    declare_parameter("Ki_steer", 0.5);
    declare_parameter("Kd_steer", 0.25);
    declare_parameter("angle_delta_max", 3.14159265/2.);
    declare_parameter("path_filename", "path.yaml");
    declare_parameter("ramp_rate", 50);
    declare_parameter("drive_min", 1550);
    declare_parameter("drive_max", 1650);
    declare_parameter("look_ahead_throttle", 6);
    declare_parameter("look_ahead_steer", 2);

    // Define parameter variables
    loop_rate = get_parameter("rate").as_double();
    cmd_max = get_parameter("cmd_max").as_int();
    cmd_min = get_parameter("cmd_min").as_int();
    max_rpm = get_parameter("max_rpm").as_double();
    wheel_diameter = get_parameter("wheel_diameter").as_double();
    gear_ratio = get_parameter("gear_ratio").as_double();
    Kp_steer = get_parameter("Kp_steer").as_double();
    Ki_steer = get_parameter("Ki_steer").as_double();
    Kd_steer = get_parameter("Kd_steer").as_double();
    angle_delta_max = get_parameter("angle_delta_max").as_double();
    path_filename = get_parameter("path_filename").as_string();
    ramp_rate = get_parameter("ramp_rate").as_int();
    drive_min = get_parameter("drive_min").as_int();
    drive_max = get_parameter("drive_max").as_int();
    look_ahead_throttle = get_parameter("look_ahead_throttle").as_int();
    look_ahead_steer = get_parameter("look_ahead_steer").as_int();
    
    // Create planned path and race_path
    planned_path = loadPathFromYamlFile();
    planned_path.header.stamp = get_clock()->now();
    planned_path.header.frame_id = "map";
    race_path.header.stamp = get_clock()->now();
    race_path.header.frame_id = "map";

    // Other variables
    race_time = 100000000000.;
    time = race_time + 10.;
    race_on = false;
    max_speed = (max_rpm / gear_ratio) * (wheel_diameter / 2.);
    cmd_neutral = (cmd_min + cmd_max) / 2;
    closest_pose_index = 0;
    steer_angle_prev = 0.;
    steer_angle_cum = 0.;
    drive_cmd_prev = cmd_neutral;
    path_size = planned_path.poses.size();

    // Publishers
    cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    planned_path_pub = create_publisher<nav_msgs::msg::Path>("path_plan", 10);
    race_path_pub = create_publisher<nav_msgs::msg::Path>("race_path", 10);
    drive_cmd_pub = create_publisher<std_msgs::msg::Int32>("drive_cmd", 10);

    // Servers
    start_race_srv = create_service<std_srvs::srv::Empty>(
      "start_race",
      std::bind(&Race_Path::start_race_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Clients
    odom_reset_cli = create_client<std_srvs::srv::Empty>("odom_reset");
    enable_drive_cli = create_client<std_srvs::srv::SetBool>("enable_drive");

    // Main timer
    int cycle_time = 1000.0 / loop_rate;
    main_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Race_Path::timer_callback, this));
      
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
  double loop_rate;
  int cmd_max, cmd_min, cmd_neutral, closest_pose_index;
  int ramp_rate, drive_min, drive_max, drive_cmd_prev;
  int look_ahead_throttle, look_ahead_steer, path_size;
  double time, race_time;
  bool race_on;
  double max_speed, wheel_diameter, max_rpm, gear_ratio;
  double Kp_steer, Ki_steer, Kd_steer, angle_delta_max;
  double steer_angle_prev, steer_angle_cum, steer_angle_der;
  geometry_msgs::msg::PoseStamped current_pose;
  nav_msgs::msg::Path planned_path, race_path;
  std::string path_filename;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr drive_cmd_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr race_path_pub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_race_srv;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr odom_reset_cli;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_drive_cli;
  rclcpp::TimerBase::SharedPtr main_timer;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;

  /// \brief The main timer callback, publishes velocity commands
  void timer_callback()
  {
    geometry_msgs::msg::Twist cmd_vel_msg;
    std_msgs::msg::Int32 drive_msg;

    // Publish goal pose and race path
    publish_race_path();
    publish_goal_path();

    // If racing, publish drive and steer commands
    if ((time < race_time) && (time >= 0)) {
      // Find current pose in path
      find_closest_pose();

      // cmd_vel commands
      cmd_vel_msg.angular.z = calculate_angular();

      // Throttle commands
      drive_msg.data = calculate_throttle();

      // Print current time
      RCLCPP_INFO(this->get_logger(), "Race Time: %f", time);
    // If before race, print countdown
    } else if (time < 0.0) {
      // Print countdown 
      RCLCPP_INFO(this->get_logger(), "Countdown: %f", -time);
      race_on = true;
      // send zero speed cmd_vel command
      cmd_vel_msg.angular.z = 0.0;
      drive_msg.data = cmd_neutral;
      // Reset race path
      nav_msgs::msg::Path empty_path;
      race_path = empty_path;
    // If after race, end race and wait
    } else {
      // send neutral throttle and cmd_vel command
      cmd_vel_msg.angular.z = 0.0;
      drive_msg.data = cmd_neutral;
      // If race just ended
      if (race_on==true) {
        // Print final time
        RCLCPP_INFO(this->get_logger(), "Race over. Race Time: %f", race_time);
        race_on = false;
        // Disable drive motor
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = false;
        int count = 0;
        while ((!enable_drive_cli->wait_for_service(1s)) && (count < 5)) {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for enable_drive service...");
          count++;
        }
        auto result = enable_drive_cli->async_send_request(request);
      }
    }

    // Update time
    time +=  1. / loop_rate;
    
    // Publish commands
    cmd_vel_pub->publish(cmd_vel_msg);
    drive_cmd_pub->publish(drive_msg);
  }

  /// \brief Updates the angular velocity command based on the current position and upcoming poses in path
  /// \return The new angular velocity command
  double calculate_angular()
  {
    double cmd_vel_steer, angle_delta;
    geometry_msgs::msg::PoseStamped target_pose;

    if ((closest_pose_index + look_ahead_steer) < path_size) {
      target_pose = planned_path.poses[closest_pose_index + look_ahead_steer];
    } else {
      target_pose = planned_path.poses[closest_pose_index];
    }

    // Convert to roll, pitch, and yaw
    double roll, pitch, yaw_target, yaw_current;
    tf2::Quaternion quaternion_target, quaternion_current;
    tf2::fromMsg(target_pose.pose.orientation, quaternion_target);
    tf2::Matrix3x3(quaternion_target).getRPY(roll, pitch, yaw_target);
    tf2::fromMsg(current_pose.pose.orientation, quaternion_current);
    tf2::Matrix3x3(quaternion_current).getRPY(roll, pitch, yaw_current);

    // Calculate angle difference and normalize to (-pi, pi), take absolute value
    angle_delta = yaw_current - yaw_target;
    while (angle_delta > 3.14159265) {
      angle_delta += -2 * 3.14159265;
    }
    while (angle_delta < -3.14159265) {
      angle_delta += 2 * 3.14159265;
    }

    // Calculate error, integral/cumulative error, derivative of error for steering
    steer_angle_cum += angle_delta * (1.0 / loop_rate);
    steer_angle_der = (angle_delta - steer_angle_prev) / (1.0 / loop_rate);
    steer_angle_prev = angle_delta;

    // Calculate steering command using PID
    cmd_vel_steer = (Kp_steer * angle_delta) + (Ki_steer * steer_angle_cum) + (Kd_steer * steer_angle_der);
    return cmd_vel_steer;
  }

  /// \brief Updates the throttle command based on the current position and upcoming poses in path
  /// \return The new drive_cmd command
  int calculate_throttle()
  {
    int drive_cmd;
    double angle_delta;
    geometry_msgs::msg::PoseStamped target_pose;

    if ((closest_pose_index + look_ahead_throttle) < path_size) {
      target_pose = planned_path.poses[closest_pose_index + look_ahead_throttle];
    } else {
      target_pose = planned_path.poses[closest_pose_index];
    }

    // Convert to roll, pitch, and yaw
    double roll, pitch, yaw_target, yaw_current;
    tf2::Quaternion quaternion_target, quaternion_current;
    tf2::fromMsg(target_pose.pose.orientation, quaternion_target);
    tf2::Matrix3x3(quaternion_target).getRPY(roll, pitch, yaw_target);
    tf2::fromMsg(current_pose.pose.orientation, quaternion_current);
    tf2::Matrix3x3(quaternion_current).getRPY(roll, pitch, yaw_current);

    // Calculate angle difference and normalize to (-pi, pi), take absolute value
    angle_delta = yaw_target - yaw_current;
    while (angle_delta > 3.14159265) {
      angle_delta += -2 * 3.14159265;
    }
    while (angle_delta < -3.14159265) {
      angle_delta += 2 * 3.14159265;
    }
    angle_delta = abs(angle_delta);

    // Calculate drive command based on angle delta
    drive_cmd = (((angle_delta_max - angle_delta) / angle_delta_max) * (drive_max - drive_min)) + drive_min;

    // Limit drive to drive max, drive neutral, and limit ramp rate
    if (drive_cmd > (drive_cmd_prev + (ramp_rate / loop_rate))) {
      drive_cmd = drive_cmd_prev + (ramp_rate / loop_rate);
    }
    if (drive_cmd > drive_max) {
      drive_cmd = drive_max;
    }
    if (drive_cmd < cmd_neutral) {
      drive_cmd = cmd_neutral;
    } 

    // Store drive cmd
    drive_cmd_prev = drive_cmd;

    // Calculate and return throttle command
    return drive_cmd;
  }

  /// \brief Updates the angular velocity command based on the current position and upcoming poses in path
  void find_closest_pose()
  {
    double dist_to_last, dist_to_check;
    geometry_msgs::msg::PoseStamped last_pose, check_pose;
    int check_index = closest_pose_index + 1;

    // Find distance to last pose in path and to next pose in path
    last_pose = planned_path.poses[closest_pose_index];
    dist_to_last = sqrt(pow((last_pose.pose.position.x - current_pose.pose.position.x), 2) + pow((last_pose.pose.position.y - current_pose.pose.position.y), 2));
    check_pose = planned_path.poses[check_index];
    dist_to_check = sqrt(pow((check_pose.pose.position.x - current_pose.pose.position.x), 2) + pow((check_pose.pose.position.y - current_pose.pose.position.y), 2));

    // If next pose is closer than last pose, check another pose ahead
    while (dist_to_check < dist_to_last) {
      // update indexes
      closest_pose_index += 1;
      check_index += 1;

      // Check next pose
      last_pose = planned_path.poses[closest_pose_index];
      dist_to_last = sqrt(pow((last_pose.pose.position.x - current_pose.pose.position.x), 2) + pow((last_pose.pose.position.y - current_pose.pose.position.y), 2));
      check_pose = planned_path.poses[check_index];
      dist_to_check = sqrt(pow((check_pose.pose.position.x - current_pose.pose.position.x), 2) + pow((check_pose.pose.position.y - current_pose.pose.position.y), 2));
    }
  }

  /// \brief Updates the robot path with the current pose and publishes the path
  void publish_race_path()
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      // Update ground truth red turtle path
      race_path.header.stamp = get_clock()->now();
      race_path.header.frame_id = "map";

      // If race_on==true, add current pose to path
      if (race_on) {
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

        // Add pose to path 
        race_path.poses.push_back(current_pose);
      }

      // Publish path
      race_path_pub->publish(race_path);

    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not transform from map to base_link: %s", ex.what());
    }
  }

  /// \brief Callback for starting the race
  void start_race_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    // Reset odometry for accurate odom measurements
    auto request_empty = std::make_shared<std_srvs::srv::Empty::Request>();
    int count = 0;
    while ((!odom_reset_cli->wait_for_service(1s)) && (count < 5)) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for reset_odom service...");
      count++;
    }
    auto result_empty = odom_reset_cli->async_send_request(request_empty);

    // Enable drive motor
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = true;
    count = 0;
    while ((!enable_drive_cli->wait_for_service(1s)) && (count < 5)) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for enable_drive service...");
      count++;
    }
    auto result = enable_drive_cli->async_send_request(request);

    // Start race
    RCLCPP_INFO(this->get_logger(), "Race Starting...");
    time = -5.0;
  }

  /// \brief Publish goal pose and path to goal
  void publish_goal_path()
  {
    // Update stamp
    planned_path.header.stamp = get_clock()->now();
    planned_path.header.frame_id = "map";

    // Publish path
    planned_path_pub->publish(planned_path);
  }

    /// \brief Convert YAML Node to geometry_msgs::msg::PoseStamped. Citation (1): ChatGPT
  geometry_msgs::msg::PoseStamped yamlToPose(const YAML::Node& node) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = node["position"]["x"].as<double>();
      pose.pose.position.y = node["position"]["y"].as<double>();
      pose.pose.position.z = node["position"]["z"].as<double>();
      pose.pose.orientation.x = node["orientation"]["x"].as<double>();
      pose.pose.orientation.y = node["orientation"]["y"].as<double>();
      pose.pose.orientation.z = node["orientation"]["z"].as<double>();
      pose.pose.orientation.w = node["orientation"]["w"].as<double>();
      return pose;
  }

  /// \brief Load and deserialize a YAML file to nav_msgs::msg::Path. Citation (1): ChatGPT
  nav_msgs::msg::Path loadPathFromYamlFile() {
      YAML::Node yamlFile = YAML::LoadFile(path_filename);
      nav_msgs::msg::Path path;
      for (const auto& node : yamlFile["poses"]) {
          path.poses.push_back(yamlToPose(node));
      }
      return path;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Race_Path>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
