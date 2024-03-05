/// \file drag_race.cpp
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

class Drag_Race : public rclcpp::Node
{
public:
  Drag_Race()
  : Node("drag_race")
  {
    // Parameters and default values
    declare_parameter("rate", 100.);
    declare_parameter("cmd_max", 2000);
    declare_parameter("cmd_min", 1000);
    declare_parameter("race_time", 0.5);
    declare_parameter("race_distance", 10.);
    declare_parameter("max_rpm", 16095.);
    declare_parameter("wheel_diameter", 0.108);
    declare_parameter("gear_ratio", 5.);
    declare_parameter("Kp", 1.0);
    declare_parameter("Ki", 0.5);
    declare_parameter("Kd", 0.25);
    declare_parameter("sample_size", 20);
    declare_parameter("sample_angle", 1.0471975512);
    declare_parameter("ramp_time", 1.5);
    declare_parameter("distance_race", false);

    // Define parameter variables
    loop_rate = get_parameter("rate").as_double();
    cmd_max = get_parameter("cmd_max").as_int();
    cmd_min = get_parameter("cmd_min").as_int();
    race_time = get_parameter("race_time").as_double();
    race_distance = get_parameter("race_distance").as_double();
    max_rpm = get_parameter("max_rpm").as_double();
    wheel_diameter = get_parameter("wheel_diameter").as_double();
    gear_ratio = get_parameter("gear_ratio").as_double();
    Kp = get_parameter("Kp").as_double();
    Ki = get_parameter("Ki").as_double();
    Kd = get_parameter("Kd").as_double();
    sample_size = get_parameter("sample_size").as_int();
    sample_angle = get_parameter("sample_angle").as_double();
    ramp_time = get_parameter("ramp_time").as_double();
    distance_race = get_parameter("distance_race").as_bool();

    // Other variables
    time = race_time + 10.;
    speed = 0.;
    race_on = false;
    max_speed = (max_rpm / gear_ratio) * (wheel_diameter / 2.);
    lidar_diff_prev = 0.;
    lidar_diff_cum = 0.;
    cmd_neutral = (cmd_min + cmd_max) / 2;

    // Publishers
    cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    planned_path_pub = create_publisher<nav_msgs::msg::Path>("path_plan", 10);
    drive_cmd_pub = create_publisher<std_msgs::msg::Int32>("drive_cmd", 10);

    // Subscribers
    odom_sub = create_subscription<nav_msgs::msg::Odometry>(
      "odom",
      10, std::bind(&Drag_Race::odom_callback, this, std::placeholders::_1));
    laser_sub = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan",
      10, std::bind(&Drag_Race::laser_callback, this, std::placeholders::_1));

    // Servers
    start_race_srv = create_service<std_srvs::srv::Empty>(
      "start_race",
      std::bind(&Drag_Race::start_race_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Clients
    imu_reset_cli = create_client<std_srvs::srv::Empty>("imu_reset");
    enable_drive_cli = create_client<std_srvs::srv::SetBool>("enable_drive");

    // Main timer
    int cycle_time = 1000.0 / loop_rate;
    main_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Drag_Race::timer_callback, this));
      
    // Transform broadcaster and listener
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    // If doing distance race, set race time very high and create goal pose
    if (distance_race) {
      race_time = 10000.;
      publish_goal_path();
    }
  }

private:
  // Initialize parameter variables
  int rate;
  double race_time, loop_rate, race_distance;
  int cmd_max, cmd_min, cmd_neutral, sample_size;
  int state = 1;
  double time, speed;
  bool race_on, distance_race;
  double max_speed, wheel_diameter, max_rpm, gear_ratio;
  double Kp, Ki, Kd, ramp_time;
  double lidar_diff_prev, lidar_diff_cum;
  double lidar_left, lidar_right, sample_angle;
  geometry_msgs::msg::PoseStamped current_pose;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr drive_cmd_pub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_race_srv;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr imu_reset_cli;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_drive_cli;
  rclcpp::TimerBase::SharedPtr main_timer;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr planned_path_pub;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;

  /// \brief The main timer callback, publishes velocity commands
  void timer_callback()
  {
    geometry_msgs::msg::Twist cmd_vel_msg;
    std_msgs::msg::Int32 drive_msg;

    if ((time < race_time) && (time >= 0)) {
      // cmd_vel commands
      cmd_vel_msg.angular.z = angular_from_lidar();

      // Ramp throttle
      if (time < ramp_time) {
        drive_msg.data = ((cmd_max - cmd_neutral) * (time / ramp_time)) + cmd_neutral;
      } else {
        drive_msg.data = cmd_max;
      }

      // If doing distance race, check if robot has passed finish line
      if (distance_race) {
        check_goal();
      }

      // Print current time
      RCLCPP_INFO(this->get_logger(), "Race Time: %f", time);
    } else if (time < 0.0) {
      // Print countdown 
      RCLCPP_INFO(this->get_logger(), "Countdown: %f", -time);
      lidar_diff_prev = 0.;
      lidar_diff_cum = 0.;
      race_on = true;
    } else if ((time >= race_time) && (time < (race_time + 5.0))) {
      // for five seconds after race ends, keep steering the car away from walls
      // send zero speed cmd_vel command
      cmd_vel_msg.angular.z = angular_from_lidar();
      drive_msg.data = 0.;
      // If race just ended
      if (race_on==true) {
        // Print final time
        RCLCPP_INFO(this->get_logger(), "Race over. Race Time: %f, Final speed: %f", race_time, speed);
        race_on = false;
        // Disable drive motor
        auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data = false;
        int count = 0;
        while ((!enable_drive_cli->wait_for_service(1s)) && (count < 5)) {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for enable_drive service...");
          count++;
        // Reset PID variables
        lidar_diff_prev = 0.;
        lidar_diff_cum = 0.;
        }
        auto result = enable_drive_cli->async_send_request(request);
      }
    } else {
      // send zero speed cmd_vel command
      cmd_vel_msg.angular.z = 0.0;
      drive_msg.data = 0.;
    }

    // update time
    time +=  1. / loop_rate;
    
    // Publish commands
    cmd_vel_pub->publish(cmd_vel_msg);
    drive_cmd_pub->publish(drive_msg);
    speed = 0.;

    // Publish goal pose and planned path if distance race
    if (distance_race) {
      publish_goal_path();
    }
  }

  /// \brief Updates the angular velocity command based on the lidar readings
  /// \return The new angular velocity command
  double angular_from_lidar()
  {
    double lidar_diff, lidar_diff_der, angular_out;

    // Calculate difference in left and right lidar reading 
    lidar_diff = -lidar_left + lidar_right;

    // Find cumulative difference and derivative of difference
    lidar_diff_cum += lidar_diff / loop_rate;
    lidar_diff_der = lidar_diff - lidar_diff_prev;

    // Store current reading
    lidar_diff_prev = lidar_diff;

    // Calculate angular_out using PID
    angular_out = (Kp * lidar_diff) + (Ki * lidar_diff_cum) + (Kd * lidar_diff_der);
    return angular_out;
  }

  /// \brief The laser callback function, the left and right lidar ranges from the scan msg
  void laser_callback(const sensor_msgs::msg::LaserScan & msg)
  {
    std::vector<float> laser_ranges;
    int left_center_index, right_center_index, full_scan_count;
    double left_sum, right_sum;
    double samples_left, samples_right;

    samples_left = sample_size;
    samples_right = sample_size;

    // Get ranges from msg
    laser_ranges = msg.ranges;
    full_scan_count = laser_ranges.size();

    // Find index to center measurements at 
    right_center_index = full_scan_count * (sample_angle / 6.28318530718);
    left_center_index = full_scan_count - right_center_index;
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "right_center_index: %i", right_center_index);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "left_center_index: %i", left_center_index);

    // Extract the data around the center points
    for(int i = (right_center_index - (sample_size / 2)); i < (right_center_index + (sample_size / 2)); i++) {
      if (!std::isinf(laser_ranges[i])) {
        right_sum += laser_ranges[i];
      } else {
        samples_right += -1;
      }
    }
    for(int i = (left_center_index - (sample_size / 2)); i < (left_center_index + (sample_size / 2)); i++) {
      if (!std::isinf(laser_ranges[i])) {
        left_sum += laser_ranges[i];
      } else {
        samples_left += -1;
      }
    }

    // Average data
    lidar_left = left_sum / samples_left;
    lidar_right = right_sum / samples_right;
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "lidar_left: %f", lidar_left);
    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "lidar_right: %f", lidar_right);
  }

  /// \brief The odometry callback function, extracts the current angular speed of the car
  void odom_callback(const nav_msgs::msg::Odometry & msg)
  {
    speed = msg.twist.twist.linear.x;
  }

  /// \brief Callback for starting the race
  void start_race_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    // Reset odometry for accurate velocity measurements
    auto request_empty = std::make_shared<std_srvs::srv::Empty::Request>();
    int count = 0;
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Resetting IMU...");
    while ((!imu_reset_cli->wait_for_service(1s)) && (count < 5)) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Waiting for reset_imu service...");
      count++;
    }
    auto result_empty = imu_reset_cli->async_send_request(request_empty);

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

  /// \brief Create a goal pose at a specified distance from the start position
  void publish_goal_path()
  {

    geometry_msgs::msg::PoseStamped goal_pose, start_pose;
    nav_msgs::msg::Path planned_path;

    // Fill out start pose
    start_pose.header.stamp = get_clock()->now();
    start_pose.header.frame_id = "map";
    start_pose.pose.position.x = 0.0;
    start_pose.pose.position.y = 0.0;
    start_pose.pose.position.z = 0.0;
    start_pose.pose.orientation.x = 0.;
    start_pose.pose.orientation.y = 0.;
    start_pose.pose.orientation.z = 0.;
    start_pose.pose.orientation.w = 1.;
    planned_path.poses.push_back(start_pose);

    // Fill out goal pose
    goal_pose.header.stamp = get_clock()->now();
    goal_pose.header.frame_id = "map";
    goal_pose.pose.position.x = race_distance;
    goal_pose.pose.position.y = 0.0;
    goal_pose.pose.position.z = 0.0;
    goal_pose.pose.orientation.x = 0.;
    goal_pose.pose.orientation.y = 0.;
    goal_pose.pose.orientation.z = 0.;
    goal_pose.pose.orientation.w = 1.;
    planned_path.poses.push_back(goal_pose);

    // Publish path
    planned_path_pub->publish(planned_path);
  }

  /// \brief Check if robot is past goal pose
  void check_goal()
  {
    geometry_msgs::msg::TransformStamped transformStamped;
    try {
      transformStamped = tf_buffer->lookupTransform("goal_pose", "base_link", tf2::TimePointZero);
    } catch (tf2::TransformException &ex) {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not transform from goal_pose to base_link: %s", ex.what());
    }
    
    if (transformStamped.transform.translation.x >= 0.) {
      race_time = time;
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Drag_Race>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
