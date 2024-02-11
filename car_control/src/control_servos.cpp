/// \file control_servos.cpp
/// \brief Controls the steering servo and ESC via a PCA 9685 over I2C
///
/// PARAMETERS:
///     rate (double): the publishing rate for wheel speed messages
///     cmd_max (int): the maximum servo command
///     timeout (double): minimum time required between receiving commands
/// SUBSCRIBES:
///     steering_cmd (std_msgs::msg::Int32): the command for the steering servo
///     drive_cmd (std_msgs::msg::Int32): the command for the drive motor ESC

#include <chrono>
#include <memory>
#include <string>
#include <random>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Control_Servos : public rclcpp::Node
{
public:
  Control_servos()
  : Node("control_servos")
  {
    // Parameters and default values
    declare_parameter("rate", 200.);
    declare_parameter("cmd_max", 180);
    declare_parameter("timeout", 1.);

    // Define parameter variables
    loop_rate = get_parameter("rate").as_double();
    cmd_max = get_parameter("cmd_max").as_int();
    timeout = get_parameter("timeout").as_double();

    // Define other variables
    default_steering_cmd = cmd_max / 2;
    default_drive_cmd = cmd_max / 2;
    steering_cmd = default_steering_cmd;
    drive_cmd = default_drive_cmd;
    now = this->get_clock()->now();
    time_now = now.seconds() + (now.nanoseconds() * 0.000000001);
    time_last_steer = time_now;
    time_last_drive = time_now;

    // Subscribers
    steering_cmd_sub = create_subscription<std_msgs::msg::Int32>(
      "steering_cmd",
      10, std::bind(&Control_Servos::steering_cmd_callback, this, std::placeholders::_1));
    drive_cmd = create_subscription<std_msgs::msg::Int32>(
      "drive_cmd",
      10, std::bind(&Control_Servos::drive_cmd_callback, this, std::placeholders::_1));

    // Main timer
    int cycle_time = 1000.0 / loop_rate;
    main_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Control_Servos::timer_callback, this));
  }

private:
  // Initialize parameter variables
  int rate;
  double loop_rate, timeout;
  int cmd_max, steering_cmd, drive_cmd, default_steering_cmd, default_drive_cmd;
  double time_now, time_last_steer, time_last_drive;
  rclcpp::Time now;

  // Initialize subscriptions and timer
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr steering_cmd_sub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr drive_cmd;
  rclcpp::TimerBase::SharedPtr main_timer;

  /// \brief The main timer callback, updates diff_drive state and publishes odom messages
  void timer_callback()
  {
    now = this->get_clock()->now();
    time_now = now.seconds() + (now.nanoseconds() * 0.000000001);

    // Check if steering and drive commands received within before timeout
    if (((time_now - time_last_steer) > timeout) || ((time_now - time_last_drive) > timeout)) {
      RCLCPP_DEBUG(this->get_logger(), "Either no steering or no drive command received within last %f seconds. Timeout.", timeout);
      steering_cmd = default_steering_cmd;
      drive_cmd = default_drive_cmd;
    }

    // Publish servo commands

    
  }

  /// \brief The steering_cmd callback function, updates the steering command
  void steering_cmd_callback(const sensor_msgs::msg::JointState & msg)
  {
    rclcpp::Time time;
    steering_cmd = msg.data;
    steering_cmd = check_command(steering_cmd);
    time = this->get_clock()->now();
    time_last_steer = time.seconds() + (time.nanoseconds() * 0.000000001);
  }

  /// \brief The drive_cmd callback function, updates the drive motor command
  void drive_cmd_callback(const sensor_msgs::msg::JointState & msg)
  {
    rclcpp::Time time;
    drive_cmd = msg.data;
    drive_cmd = check_command(drive_cmd);
    time = this->get_clock()->now();
    time_last_drive = time.seconds() + (time.nanoseconds() * 0.000000001);
  }

  /// \brief Checks if a servo command is within a valid range
  /// \param int The command to be checked
  /// \returns the command constrained to the valid command range
  int check_command(int cmd)
  {
    // Contrain the command to the valid range
    if (cmd < 0) {
        cmd = 0;
    } elif (cmd > cmd_max) {
        cmd = cmd_max;
    }
    // Return the constrained command
    return cmd;
  }
};



int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Read_Encoder>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
