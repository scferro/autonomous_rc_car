/// \file path_race.cpp
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

class Path_Race : public rclcpp::Node
{
public:
  Path_Race()
  : Node("path_race")
  {
    // Parameters and default values
    declare_parameter("rate", 100.);
    declare_parameter("cmd_max", 2000);
    declare_parameter("cmd_min", 1000);
    declare_parameter("max_rpm", 16095.);
    declare_parameter("wheel_diameter", 0.108);
    declare_parameter("gear_ratio", 5.);
    declare_parameter("Kp", 1.0);
    declare_parameter("Ki", 0.5);
    declare_parameter("Kd", 0.25);

    // Define parameter variables
    loop_rate = get_parameter("rate").as_double();
    cmd_max = get_parameter("cmd_max").as_int();
    cmd_min = get_parameter("cmd_min").as_int();
    max_rpm = get_parameter("max_rpm").as_double();
    wheel_diameter = get_parameter("wheel_diameter").as_double();
    gear_ratio = get_parameter("gear_ratio").as_double();
    Kp = get_parameter("Kp").as_double();
    Ki = get_parameter("Ki").as_double();
    Kd = get_parameter("Kd").as_double();

    // Other variables
    time = race_time + 10.;
    race_on = false;
    max_speed = (max_rpm / gear_ratio) * (wheel_diameter / 2.);
    cmd_neutral = (cmd_min + cmd_max) / 2;
    closest_pose_index = 0;
    
    // Create planned path and race_path
    planned_path.header.stamp = get_clock()->now();
    planned_path.header.frame_id = "map";
    race_path.header.stamp = get_clock()->now();
    race_path.header.frame_id = "map";

    // Publishers
    cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    planned_path_pub = create_publisher<nav_msgs::msg::Path>("path_plan", 10);
    race_path_pub = create_publisher<nav_msgs::msg::Path>("path_race", 10);
    drive_cmd_pub = create_publisher<std_msgs::msg::Int32>("drive_cmd", 10);

    // Subscribers
    odom_sub = create_subscription<nav_msgs::msg::Odometry>(
      "odom",
      10, std::bind(&Path_Race::odom_callback, this, std::placeholders::_1));

    // Servers
    start_race_srv = create_service<std_srvs::srv::Empty>(
      "start_race",
      std::bind(&Path_Race::start_race_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Clients
    odom_reset_cli = create_client<std_srvs::srv::Empty>("odom_reset");
    enable_drive_cli = create_client<std_srvs::srv::SetBool>("enable_drive");

    // Main timer
    int cycle_time = 1000.0 / loop_rate;
    main_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Path_Race::timer_callback, this));
      
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
  double time;
  bool race_on;
  double max_speed, wheel_diameter, max_rpm, gear_ratio;
  double Kp, Ki, Kd;
  geometry_msgs::msg::PoseStamped current_pose;
  nav_msgs::msg::Path planned_path, race_path;

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

    // Publish goal pose and race path
    publish_race_path();
    publish_goal_path();
  }

  /// \brief Updates the angular velocity command based on the current position and upcoming poses in path
  /// \return The new angular velocity command
  double calculate_angular()
  {

  }

  /// \brief Updates the throttle command based on the current position and upcoming poses in path
  /// \return The new drive_cmd command
  double calculate_throttle()
  {

  }

  /// \brief Updates the angular velocity command based on the current position and upcoming poses in path
  void find_closest_pose()
  {

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
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Path_Race>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
