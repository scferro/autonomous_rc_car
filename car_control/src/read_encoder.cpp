/// \file read_encoder.cpp
/// \brief Reads the encoder and publishes the wheel speed
///
/// PARAMETERS:
///     rate (double): the publishing rate for wheel speed messages
/// PUBLISHES:
///     wheel_speed (std_msgs::msg::Float64): the speed of the rear wheels

#include <chrono>
#include <memory>
#include <string>
#include <random>
#include <vector>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class Read_Encoder : public rclcpp::Node
{
public:
  Read_Encoder()
  : Node("read_encoder")
  {
    // Parameters and default values
    declare_parameter("rate", 200.);
    declare_parameter("encoder_ticks", 8192);
    declare_parameter("gear_ratio", 5.);

    // Define parameter variables
    loop_rate = get_parameter("rate").as_double();
    encoder_ticks = get_parameter("encoder_ticks").as_int();
    gear_ratio = get_parameter("gear_ratio").as_double();

    // Define other variables
    wheel_speed = 0.0;

    // Publishers
    wheel_speed_pub = create_publisher<std_msgs::msg::Float64>("wheel_speed", 10);

    // Main timer
    int cycle_time = 1000.0 / loop_rate;
    main_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Read_Encoder::timer_callback, this));
  }

private:
  // Initialize parameter variables
  int rate;
  double loop_rate, gear ratio;
  int encoder_ticks;
  double wheel_speed;

  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr wheel_speed_pub;
  rclcpp::TimerBase::SharedPtr main_timer;

  /// \brief The main timer callback, updates diff_drive state and publishes odom messages
  void timer_callback()
  {
    std_msgs::msg::Float64 wheel_speed_msg;


    // Publish wheel speed messages
    wheel_speed_msg.data = wheel_speed;
    wheel_speed_pub->publish(wheel_speed_msg);
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
