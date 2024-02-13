/// \file mode_node.cpp
/// \brief Tracks the current mode of the vehicle and publishes mode messages
///
/// PARAMETERS:
///     rate (double): the publishing rate for wheel speed messages (Hz)
///     initial_mode (int): the mode the car should start in
///     max_mode (gear int): the maximum valid mode
/// PUBLISHES:
///     mode (std_msgs::msg::Int32): the current mode
/// SERVERS:
///     mode (rc_car_interfaces::srv::SetMode): sets the current mode of the car

/// DESCRIPTION OF MODES
/// NUMBER   COLOR   FUNCTION
/// 0        GREEN   Manual control ON
/// 1        RED     Robot stopped
/// 2        BLUE    
/// 3        YELLOW  

#include <chrono>
#include <memory>
#include <string>
#include <random>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "rc_car_interfaces/srv/set_mode.hpp"

using namespace std::chrono_literals;

class Mode_Node : public rclcpp::Node
{
public:
  Mode_Node()
  : Node("mode_node")
  {
    // Parameters and default values
    declare_parameter("rate", 10.);
    declare_parameter("initial_mode", 0);
    declare_parameter("max_mode", 3);

    // Define parameter variables
    loop_rate = get_parameter("rate").as_double();
    initial_mode = get_parameter("initial_mode").as_int();
    mode = get_parameter("max_mode").as_int();

    // Define other variables

    // Publishers
    mode_pub = create_publisher<std_msgs::msg::Int32>("wheel_speed", 10);

    // Services
    mode_srv = create_service<rc_car_interfaces::srv::SetMode>(
      "set_mode",
      std::bind(
        &Mode_Node::mode_srv_callback, this, std::placeholders::_1,
        std::placeholders::_2));

    // Main timer
    int cycle_time = 1000.0 / loop_rate;
    main_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Mode_Node::timer_callback, this));
  }

private:
  // Initialize parameter variables
  int rate;
  double loop_rate;
  int mode, max_mode, initial_mode;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mode_pub;
  rclcpp::TimerBase::SharedPtr main_timer;
  rclcpp::Service<rc_car_interfaces::srv::SetMode>::SharedPtr mode_srv;

  /// \brief The main timer callback, updates diff_drive state and publishes odom messages
  void timer_callback()
  {
    std_msgs::msg::Int32 mode_msg;

    // Publish wheel speed messages
    mode_msg.data = mode;
    mode_pub->publish(mode_msg);
  }

  /// \brief Set the current mode of the robot
  /// \param request The desired mode of the robot
  void mode_srv_callback(
    rc_car_interfaces::srv::SetMode::Request::SharedPtr request,
    rc_car_interfaces::srv::SetMode::Response::SharedPtr)
  {
    // Extract desired x, y, theta
    mode = request->data;

    if ((mode > max_mode) || (mode < 0)) {
        mode = initial_mode;
        RCLCPP_INFO(this->get_logger(), "Mode request invalid. Reset to MODE %i", mode);
    } else {
        RCLCPP_INFO(this->get_logger(), "Mode set to MODE %i", mode);
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Mode_Node>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
