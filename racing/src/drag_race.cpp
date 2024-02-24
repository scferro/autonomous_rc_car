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

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_srvs/srv/empty.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "nav_msgs/msg/odometry.hpp"

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
    declare_parameter("race_time", 5.);
    declare_parameter("max_rpm", 16095.);
    declare_parameter("wheel_diameter", 0.108);
    declare_parameter("gear_ratio", 5.);

    // Define parameter variables
    loop_rate = get_parameter("rate").as_double();
    cmd_max = get_parameter("cmd_max").as_int();
    race_time = get_parameter("race_time").as_double();
    max_rpm = get_parameter("max_rpm").as_double();
    wheel_diameter = get_parameter("wheel_diameter").as_double();
    gear_ratio = get_parameter("gear_ratio").as_double();

    // Other variables
    time = race_time + 1.;
    speed = 0.;
    race_on = false;
    max_speed = (max_rpm / gear_ratio) * (wheel_diameter / 2.);

    // Publishers
    cmd_vel_pub = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    // Subscribers
    odom_sub = create_subscription<nav_msgs::msg::Odometry>(
      "odom",
      10, std::bind(&Drag_Race::odom_callback, this, std::placeholders::_1));

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
  }

private:
  // Initialize parameter variables
  int rate;
  double angular_velocity, race_time, loop_rate;
  int cmd_max;
  int state = 1;
  double time, speed;
  bool race_on;
  double max_speed, wheel_diameter, max_rpm, gear_ratio;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr start_race_srv;
  rclcpp::Client<std_srvs::srv::Empty>::SharedPtr imu_reset_cli;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr enable_drive_cli;
  rclcpp::TimerBase::SharedPtr main_timer;

  /// \brief The main timer callback, publishes velocity commands
  void timer_callback()
  {
    geometry_msgs::msg::Twist cmd_vel_msg;

    if ((time < race_time) && (time >= 0)) {
      race_on = true;

      // cmd_vel and servo commands
      cmd_vel_msg.angular.z = 0.0;
      cmd_vel_msg.linear.x = max_speed;
      
      // Print current speed
      RCLCPP_INFO(this->get_logger(), "Racing! Current speed: %f", speed);
    } else if (time < 0.0) {
      // Print countdown 
      RCLCPP_INFO(this->get_logger(), "Countdown: %f", -time);
    } else {
      // send zero speed cmd_vel command
      cmd_vel_msg.angular.z = 0.0;
      cmd_vel_msg.linear.x = 0.0;

      // If race just ended
      if (race_on==true) {
        // Print final time
        RCLCPP_INFO(this->get_logger(), "Race over. Final speed: %f", speed);
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

    // update time
    time +=  1. / loop_rate;
    
    // Publish commands
    cmd_vel_pub->publish(cmd_vel_msg);
    speed = 0.;
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
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Drag_Race>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
