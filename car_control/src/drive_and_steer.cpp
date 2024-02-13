/// \file drive_and_steer.cpp
/// \brief Controls the car to drive and steer at a specified velocity
///
/// PARAMETERS:
///     loop_rate (double): the publishing rate of the main loop (Hz)
///     cmd_max (int): the maximum command sent to the servo
///     cmd_min (int): the minimum command sent to the servo
///     use_wheel_speed (bool): if the wheel encoder should be used for robot speed feedback
///     wheel_diameter (double): the publishing rate of the main loop (Hz)
///     timeout (double): minimum time required between receiving commands (s)
/// SUBSCRIBES:
///     cmd_vel (geometry_msgs::msg::Twist): the robot velocity commands
///     wheel_speed (std_msgs::msg::Float64): the speed of the rear wheels
/// PUBLISHES:
///     steering_cmd (std_msgs::msg::Int32): the command for the steering servo
///     drive_cmd (std_msgs::msg::Int32): the command for the drive motor ESC

#include <chrono>
#include <memory>
#include <string>
#include <random>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

// Used ChatGPT for debugging
// Refer to Citation [5] ChatGPT

using namespace std::chrono_literals;

class Drive_and_Steer : public rclcpp::Node
{
public:
  Drive_and_Steer()
  : Node("drive_and_steer")
  {
    // Parameters and default values
    declare_parameter("loop_rate", 100.);
    declare_parameter("cmd_max", 2000);
    declare_parameter("cmd_min", 1000);
    declare_parameter("use_wheel_speed", false);
    declare_parameter("wheel_diameter", 0.108);
    declare_parameter("timeout", 1.);
    
    // Define parameter variables
    loop_rate = get_parameter("loop_rate").as_double();
    cmd_max = get_parameter("cmd_max").as_int();
    cmd_min = get_parameter("cmd_min").as_int();
    use_wheel_speed = get_parameter("use_wheel_speed").as_bool();
    wheel_diameter = get_parameter("wheel_diameter").as_double();
    timeout = get_parameter("timeout").as_double();

    // Define other variables
    P_l = 0.0;
    I_l = 0.0;
    D_l = 0.0;
    P_a = 1.0;
    I_a = 0.0;
    D_a = 0.0;
    cmd_neutral = (cmd_min + cmd_max) / 2;
    linear_vel = 0.;
    linear_vel_cmd = 0.;
    linear_error = 0.;
    linear_error_prev = 0.;
    linear_error_cum = 0.;
    linear_error_der = 0.;
    angular_vel = 0.;
    angular_vel_cmd = 0.;
    angular_error = 0.;
    angular_error_prev = 0.;
    angular_error_cum = 0.;
    angular_error_der = 0.;
    now = this->get_clock()->now();
    time_now = now.seconds() + (now.nanoseconds() * 0.000000001);
    time_last = time_now;

    // Publishers
    steering_cmd_pub = create_publisher<std_msgs::msg::Int32>("steering_cmd", 10);
    drive_cmd_pub = create_publisher<std_msgs::msg::Int32>("drive_cmd", 10);

    // Subscribers
    cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel",
      10, std::bind(&Drive_and_Steer::cmd_vel_callback, this, std::placeholders::_1));
    wheel_speed_sub = create_subscription<std_msgs::msg::Float64>(
      "wheel_speed",
      10, std::bind(&Drive_and_Steer::wheel_speed_callback, this, std::placeholders::_1));

    // Main timer
    int cycle_time = 1000.0 / loop_rate;
    main_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Drive_and_Steer::timer_callback, this));
  }

private:
  // Initialize parameter variables
  int rate;
  int loop_rate, cmd_max, cmd_min, cmd_neutral;
  int drive_cmd, steer_cmd;
  bool use_wheel_speed;
  double wheel_diameter, timeout;
  double P_l, I_l, D_l, P_a, I_a, D_a;
  double linear_vel, angular_vel, linear_vel_cmd, angular_vel_cmd;
  double linear_error, linear_error_prev, linear_error_cum, linear_error_der;
  double angular_error, angular_error_prev, angular_error_cum, angular_error_der;
  double time_now, time_last;
  rclcpp::Time now;
  
  // Create ROS publishers, timers, broadcasters, etc.
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr steering_cmd_pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr drive_cmd_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr wheel_speed_sub;
  rclcpp::TimerBase::SharedPtr main_timer;

  /// \brief The main timer callback, loop for both wheels
  void timer_callback()
  {
    // Initialize messages
    std_msgs::msg::Int32 drive_msg, steer_msg;

    // Calculate error
    linear_error = linear_vel_cmd - linear_vel;
    angular_error = angular_vel_cmd - angular_vel;
    
    // Calculate integral/cumulative error
    linear_error_cum += linear_error * (1.0 / loop_rate);
    angular_error_cum += angular_error * (1.0 / loop_rate);
    
    // Calculate derivative of error
    linear_error_der = (linear_error - linear_error_prev) / (1.0 / loop_rate);
    angular_error_der = (angular_error - angular_error_prev) / (1.0 / loop_rate);

    // Calculate commands
    drive_cmd = (P_l * linear_error) + (I_l * linear_error_cum) + (D_l * linear_error_der);
    steer_cmd = (P_a * angular_error) + (I_a * angular_error_cum) + (D_a * angular_error_der);

    // Limiting servo commands and adding to message
    drive_msg.data = limit_cmd(drive_cmd);
    steer_msg.data = limit_cmd(steer_cmd);

    // Get time
    now = this->get_clock()->now();
    time_now = now.seconds() + (now.nanoseconds() * 0.000000001);

    // Check if cmd_vel received before timeout
    if ((time_now - time_last) > timeout) {
      RCLCPP_DEBUG(this->get_logger(), "No cmd_vel command received within last %f seconds. Timeout.", timeout);

      // Setting servo commands to stop the car
      drive_msg.data = cmd_neutral;
      steer_msg.data = cmd_neutral;
    }

    // Store errors as previous errors
    linear_error_prev = linear_error;
    angular_error_prev = angular_error;
    
    // Publish command messages
    drive_cmd_pub->publish(drive_msg);
    steering_cmd_pub->publish(steer_msg);

  }

  /// \brief The cmd_vel callback function, publishes motor speed commands based on received twist command
  void cmd_vel_callback(const geometry_msgs::msg::Twist & msg)
  {
    rclcpp::Time time;
    linear_vel_cmd = msg.linear.x;
    angular_vel_cmd = msg.angular.z;
    time = this->get_clock()->now();
    time_last = time.seconds() + (time.nanoseconds() * 0.000000001);
  }

  /// \brief The wheel_speed callback function, calculates vehicle speed based on wheel speed if enabled
  void wheel_speed_callback(const std_msgs::msg::Float64 & msg)
  {
    // Set the linear velocity using wheel speed if use_wheel_speed is set to true
    if (use_wheel_speed==true) {
    linear_vel = msg.data * wheel_diameter / 2;
    }
  }

  /// \brief Limits the range of a cmd to [-motor_cmd_max, motor_cmd_max]
  int limit_cmd(int cmd)
  {
    if (cmd > cmd_max) {
      cmd = cmd_max;
    } else if (cmd < cmd_min) {
      cmd = cmd_min;
    }
    return cmd;
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Drive_and_Steer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
