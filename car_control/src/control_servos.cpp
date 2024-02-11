/// \file control_servos.cpp
/// \brief Controls the steering servo and ESC via a PCA 9685 over I2C
///
/// PARAMETERS:
///     rate (double): the publishing rate for wheel speed messages
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

    // Define parameter variables
    loop_rate = get_parameter("rate").as_double();
    cmd_max = get_parameter("cmd_max").as_int();

    // Define other variables

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
  double loop_rate;
  int cmd_max;

  // Initialize subscriptions and timer
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr steering_cmd_sub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr drive_cmd;
  rclcpp::TimerBase::SharedPtr main_timer;

  /// \brief The main timer callback, updates diff_drive state and publishes odom messages
  void timer_callback()
  {
    
  }

  /// \brief The joint_state callback function, updates the stored wheel position
  void steering_cmd_callback(const sensor_msgs::msg::JointState & msg)
  {
    
  }

  /// \brief The joint_state callback function, updates the stored wheel position
  void drive_cmd_callback(const sensor_msgs::msg::JointState & msg)
  {
    
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
