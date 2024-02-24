/// \file odometry.cpp
/// \brief Takes in data from the RealSense IMU and wheel encoders to provide the current robot twist
///
/// PARAMETERS:
///     loop_rate (double): the publishing rate of the main loop (Hz)
///     encoder_rate (double): the rate to read the encoder (Hz)
///     encoder_ticks (int): ticks per revolution for the encoder
///     gear_ratio (double): the gear ratio of the robot
///     wheel_diameter (double): the diameter fo the wheel (m)
///     alpha (double): the blending coefficient for blending gyro and accel angle estimates, % of accel used
///     beta (double): the blending coefficient for blending IMU and wheel speed odom, % of wheelspeed used
///     gyro_thresh (double): the minimum value to read from the gyro, values below this will be read as 0.
///     simulate (bool): determines if wheel speed odom data should be collected from the real encoder or simulation data
/// SUBSCRIBES:
///     camera/camera/gyro/sample (sensor_msgs::msg::Imu): the gyro data
///     camera/camera/accel/sample (sensor_msgs::msg::Imu): the accelerometer data
/// PUBLISHES:
///     odom (nav_msgs::msg::Odometry): odometry using the IMU blended with wheel speed data (considered most accurate)
///     odom_encoder (nav_msgs::msg::Odometry): odometry the wheel speed encoder only
/// SERVERS:
///     reset_imu (std_srvs::srv::Empty): resets IMU data

#include <chrono>
#include <memory>
#include <string>
#include <random>
#include <vector>
#include <iostream>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <cstring>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;

class AS5048A {
public:
    AS5048A(const std::string& spiDevice, uint8_t spiMode, uint32_t spiSpeed, uint8_t spiBitsPerWord)
        : fd(-1), mode(spiMode), speed(spiSpeed), bits(spiBitsPerWord) {
        // Open SPI device
        fd = open(spiDevice.c_str(), O_RDWR);
        if (fd < 0) {
            //RCLCPP_INFO(this->get_logger(), "Can't open device.");
            return;
        }

        // SPI mode
        ioctl(fd, SPI_IOC_WR_MODE, &mode);
        ioctl(fd, SPI_IOC_RD_MODE, &mode);

        // Bits per word
        ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
        ioctl(fd, SPI_IOC_RD_BITS_PER_WORD, &bits);

        // Max speed Hz
        ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed);
        ioctl(fd, SPI_IOC_RD_MAX_SPEED_HZ, &speed);
    }

    ~AS5048A() {
        if (fd >= 0) {
            close(fd);
        }
    }

    uint16_t readAngle() {
        uint8_t tx[] = {0xFF, 0xFF}; // Dummy bytes to trigger clock
        uint8_t rx[2] = {0, 0};
        struct spi_ioc_transfer tr = {
            .tx_buf = (unsigned long)tx,
            .rx_buf = (unsigned long)rx,
            .len = 2,
            .speed_hz = speed,
            .delay_usecs = 0,
            .bits_per_word = bits,
        };

        if (ioctl(fd, SPI_IOC_MESSAGE(1), &tr) < 1) {
            //RCLCPP_INFO(this->get_logger(), "Can't send spi message.");
            return 0;
        }

        // Combine the two bytes received into one 16-bit value and mask out the first two bits
        uint16_t angle = ((rx[0] << 8) | rx[1]) & 0x1FFF;

        return angle;
    }

private:
    int fd;
    uint8_t mode;
    uint32_t speed;
    uint8_t bits;
};

class Odometry : public rclcpp::Node
{
public:
  Odometry()
  : Node("odometry")
  {
    // Parameters and default values
    declare_parameter("loop_rate", 100.);
    declare_parameter("encoder_rate", 200.);
    declare_parameter("encoder_ticks", 8192);
    declare_parameter("gear_ratio", 5.);
    declare_parameter("wheel_diameter", 0.108);
    declare_parameter("alpha", 0.05);
    declare_parameter("beta", 0.00);
    declare_parameter("gyro_thresh", 0.02);
    declare_parameter("simulate", true);
    
    // Define parameter variables
    loop_rate = get_parameter("loop_rate").as_double();
    encoder_rate = get_parameter("encoder_rate").as_double();
    encoder_ticks = get_parameter("encoder_ticks").as_int();
    gear_ratio = get_parameter("gear_ratio").as_double();
    wheel_diameter = get_parameter("wheel_diameter").as_double();
    alpha = get_parameter("alpha").as_double();
    beta = get_parameter("beta").as_double();
    gyro_thresh = get_parameter("gyro_thresh").as_double();
    simulate = get_parameter("simulate").as_bool();

    // Define other variables
    linear_vel = 0.0;
    linear_accel = 0.0;
    angular_vel = 0.0;
    angle_prev = 0.;
    raw_accel[0] = 0.00001;
    raw_accel[1] = 0.00001;
    raw_accel[2] = 0.00001;
    raw_gyro[0] = 0.00001;
    raw_gyro[1] = 0.00001;
    raw_gyro[2] = 0.00001;
    angles[0] = 0.0;
    angles[1] = 0.0;
    angles[2] = 0.0;
    gyro_angles[0] = 0.0;
    gyro_angles[1] = 0.0;
    gyro_angles[2] = 0.0;
    accel_angles[0] = 0.0;
    accel_angles[1] = 0.0;
    accel_angles[2] = 0.0;
    chassis_speed = 0.0;

    // Define custom QoS profile
    rclcpp::QoS custom_qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    custom_qos_profile.best_effort();
    custom_qos_profile.durability_volatile();

    // Publishers
    odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    odom_encoder_pub = create_publisher<nav_msgs::msg::Odometry>("odom_encoder", 10);

    // Subscribers
    accel_sub = create_subscription<sensor_msgs::msg::Imu>(
      "camera/camera/accel/sample",
      custom_qos_profile, std::bind(&Odometry::accel_callback, this, std::placeholders::_1));
    gyro_sub = create_subscription<sensor_msgs::msg::Imu>(
      "camera/camera/gyro/sample",
      custom_qos_profile, std::bind(&Odometry::gyro_callback, this, std::placeholders::_1));
    joint_states_sub = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states_sim",
      10, std::bind(&Odometry::joint_states_callback, this, std::placeholders::_1));

    // Servers
    imu_reset_srv = create_service<std_srvs::srv::Empty>(
      "imu_reset",
      std::bind(&Odometry::imu_reset_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Timers
    int cycle_time = 1000.0 / loop_rate;
    int encoder_cycle_time = 1000.0 / encoder_rate;
    encoder_timer = this->create_wall_timer(
      std::chrono::milliseconds(encoder_cycle_time),
      std::bind(&Odometry::encoder_timer_callback, this));
    imu_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Odometry::imu_timer_callback, this));
  }

private:
  // Initialize parameter variables
  int _rate;
  double raw_accel[3], accel[3], raw_gyro[3];
  double angles[3], gyro_angles[3], accel_angles[3];
  double linear_vel, linear_accel, angular_vel;
  int rate;
  double loop_rate, encoder_rate, gear_ratio, alpha, beta;
  int encoder_ticks;
  uint16_t angle, angle_prev;
  double delta, wheel_diameter, chassis_speed;
  double gyro_thresh, wheel_speed_sim;
  bool simulate;

  // Initialize encoder object 
  AS5048A encoder = AS5048A("/dev/spidev0.0", SPI_MODE_1, 1000000, 8); // Adjust as necessary

  // Create ROS publishers, timers, broadcasters, etc.
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_encoder_pub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gyro_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr accel_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr imu_reset_srv;
  rclcpp::TimerBase::SharedPtr encoder_timer;
  rclcpp::TimerBase::SharedPtr imu_timer;

  /// \brief The main timer callback, loop for both wheels
  void imu_timer_callback()
  {
    nav_msgs::msg::Odometry odom_msg;

    // Calculate current gyro angles
    gyro_angles[0] = raw_gyro[0] / loop_rate + angles[0];
    gyro_angles[1] = raw_gyro[1] / loop_rate + angles[1];
    gyro_angles[2] = raw_gyro[2] / loop_rate + angles[2];

    // Calculate current accel angles
    accel_angles[0] = atan(-raw_accel[2] / sqrt(pow(raw_accel[0], 2) + pow(raw_accel[1], 2)));
    accel_angles[2] = atan(raw_accel[0] / sqrt(pow(raw_accel[2], 2) + pow(raw_accel[1], 2)));

    // Blend angles with complementary filter
    angles[0] = (gyro_angles[0] * (1. - alpha)) + (accel_angles[0] * alpha);
    angles[1] = gyro_angles[1];
    angles[2] = (gyro_angles[2] * (1. - alpha)) + (accel_angles[2] * alpha);

    // RCLCPP_INFO(this->get_logger(), "raw_accel[0]: %f", raw_accel[0]);
    // RCLCPP_INFO(this->get_logger(), "raw_accel[1]: %f", raw_accel[1]);
    // RCLCPP_INFO(this->get_logger(), "raw_accel[2]: %f", raw_accel[2]);

    // Calculate current forward velocity based on gyro angles and accel
    linear_accel = (raw_accel[1] * sin(angles[1])) + (raw_accel[2] * cos(angles[1]));
    // vertical_accel = ((raw_accel[0] * cos(angles[0]) * sin(angles[2])) + (raw_accel[1] * cos(angles[0]) * cos(angles[2])) + (-raw_accel[2] * sin(angles[0]) * cos(angles[2])));

    // Find current linear and angular velocity
    linear_vel += linear_accel / loop_rate;
    angular_vel = raw_gyro[1];

    // Blend linear_vel with chassis speed from encoder
    linear_vel = (chassis_speed * beta) + (linear_vel * (1. - beta));

    // Add velocities to twist message
    odom_msg.twist.twist.linear.x = linear_vel;
    odom_msg.twist.twist.angular.z = angular_vel;
    odom_msg.pose.pose.orientation.x = angles[0];
    odom_msg.pose.pose.orientation.y = angles[1];
    odom_msg.pose.pose.orientation.z = angles[2];

    // Publish odom_msg
    odom_pub->publish(odom_msg);
  }

  /// \brief The main timer callback, updates diff_drive state and publishes odom messages
  void encoder_timer_callback()
  {
    nav_msgs::msg::Odometry odom_msg;
    double wheel_speed;

    // Check if if using simulation or not
    if (simulate==true) {
      // If using simulation, read joint state messages to find wheel speed
      wheel_speed = wheel_speed_sim;      
    } else {
      // If not using simulation, read encoder for wheel speed
      angle = encoder.readAngle();

      // Calculate change since last reading
      delta = angle - angle_prev;
      if (abs(delta) > encoder_ticks/2) {
        if (abs(delta)==delta) {
          delta = -(encoder_ticks - abs(delta));
        } else {
          delta = (encoder_ticks - abs(delta));
        }
      }

      // Calculate wheel_speed
      wheel_speed = (delta / (encoder_ticks * gear_ratio)) * loop_rate * 6.283185307;
    }

    // Calculate chassis speed based on wheel motor
    chassis_speed = wheel_speed * wheel_diameter / 2;

    // Publish wheel speed messages
    odom_msg.twist.twist.linear.x = chassis_speed;
    odom_encoder_pub->publish(odom_msg);

    // Set previous angle
    angle_prev = angle;
  }

  /// \brief The gyro callback function, stores data published by the IMU
  void gyro_callback(const sensor_msgs::msg::Imu & msg)
  {
    if (simulate==false) {
      raw_gyro[0] = msg.angular_velocity.x;
      raw_gyro[1] = msg.angular_velocity.y;
      raw_gyro[2] = msg.angular_velocity.z;
    } else {
      raw_gyro[0] = msg.angular_velocity.z;
      raw_gyro[1] = msg.angular_velocity.x;
      raw_gyro[2] = msg.angular_velocity.y;
    }

    for (int i = 0; i < 3; i++) {
        if ((raw_gyro[i] < gyro_thresh && (raw_gyro[i] > -gyro_thresh))) {
          raw_gyro[i] = 0.0;
        }
    }
  }

  /// \brief The accel callback function, stores data published by the IMU
  void accel_callback(const sensor_msgs::msg::Imu & msg)
  {
    if (simulate==false) {
      raw_accel[0] = msg.linear_acceleration.x;
      raw_accel[1] = msg.linear_acceleration.y;
      raw_accel[2] = msg.linear_acceleration.z;
    } else {
      raw_accel[0] = msg.linear_acceleration.z;
      raw_accel[1] = msg.linear_acceleration.x;
      raw_accel[2] = -msg.linear_acceleration.y;
    }
  }

  /// \brief The joint_states callback function, stores the wheel speed of the rear wheels
  void joint_states_callback(const sensor_msgs::msg::JointState & msg)
  {
    wheel_speed_sim = msg.velocity[2];
  }

  /// \brief Callback reseting IMU odometry, should only be used when robot is stationary and level
  void imu_reset_callback(
    std_srvs::srv::Empty::Request::SharedPtr,
    std_srvs::srv::Empty::Response::SharedPtr)
  {
    // Reset all IMU variables
    linear_vel = 0.0;
    linear_accel = 0.0;
    angular_vel = 0.0;
    angle_prev = 0.;
    angles[0] = 0.0;
    angles[1] = 0.0;
    angles[2] = 0.0;
    gyro_angles[0] = 0.0;
    gyro_angles[1] = 0.0;
    gyro_angles[2] = 0.0;
    accel_angles[0] = 0.0;
    accel_angles[1] = 0.0;
    accel_angles[2] = 0.0;
    chassis_speed = 0.0;

    RCLCPP_INFO(this->get_logger(), "IMU Reset.");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Odometry>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
