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
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "tf2_ros/transform_listener.h"
#include "nav_msgs/msg/path.hpp"
#include "tf2_ros/buffer.h"

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
        uint16_t angle = ((rx[0] << 8) | rx[1]) & 0x3FFF;

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
    declare_parameter("loop_rate", 200.);
    declare_parameter("encoder_rate", 1000.);
    declare_parameter("encoder_rate_sim", 100.);
    declare_parameter("encoder_ticks", 16384);
    declare_parameter("gear_ratio", 5.);
    declare_parameter("wheel_diameter", 0.108);
    declare_parameter("alpha", 0.02);
    declare_parameter("beta", 0.02);
    declare_parameter("gyro_thresh", 0.1);
    declare_parameter("accel_thresh", 0.05);
    declare_parameter("vel_thresh", 0.01);
    declare_parameter("path_rate", 10.);
    declare_parameter("simulate", false);
    declare_parameter("publish_path", false);
    
    // Define parameter variables
    loop_rate = get_parameter("loop_rate").as_double();
    encoder_rate = get_parameter("encoder_rate").as_double();
    encoder_rate_sim = get_parameter("encoder_rate_sim").as_double();
    encoder_ticks = get_parameter("encoder_ticks").as_int();
    gear_ratio = get_parameter("gear_ratio").as_double();
    wheel_diameter = get_parameter("wheel_diameter").as_double();
    alpha = get_parameter("alpha").as_double();
    beta = get_parameter("beta").as_double();
    gyro_thresh = get_parameter("gyro_thresh").as_double();
    accel_thresh = get_parameter("accel_thresh").as_double();
    vel_thresh = get_parameter("vel_thresh").as_double();
    path_rate = get_parameter("path_rate").as_double();
    simulate = get_parameter("simulate").as_bool();
    publish_path = get_parameter("publish_path").as_bool();

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
    wheel_speed_sim_prev = 0.;
    x_pos = 0.;
    y_pos = 0.;
    theta = 0.;
    theta_prev = 0.;

    // Set encoder loop to a slower speed if simulating
    if (simulate) {
      encoder_rate = encoder_rate_sim;
    }

    // Define custom QoS profile
    rclcpp::QoS custom_qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
    custom_qos_profile.best_effort();
    custom_qos_profile.durability_volatile();

    // Publishers
    odom_pub = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    odom_encoder_pub = create_publisher<nav_msgs::msg::Odometry>("odom_encoder", 10);
    laser_pub = create_publisher<sensor_msgs::msg::LaserScan>("scan", 10);
    joint_states_pub = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    path_pub = create_publisher<nav_msgs::msg::Path>("path", 10);

    // Subscribers
    accel_sub = create_subscription<sensor_msgs::msg::Imu>(
      "camera/camera/accel/sample",
      custom_qos_profile, std::bind(&Odometry::accel_callback, this, std::placeholders::_1));
    gyro_sub = create_subscription<sensor_msgs::msg::Imu>(
      "camera/camera/gyro/sample",
      custom_qos_profile, std::bind(&Odometry::gyro_callback, this, std::placeholders::_1));
    joint_states_sub = create_subscription<sensor_msgs::msg::JointState>(
      "joint_states_sim",
      10, std::bind(&Odometry::joint_states_sim_callback, this, std::placeholders::_1));
    laser_sim_sub = create_subscription<sensor_msgs::msg::LaserScan>(
      "scan_sim",
      10, std::bind(&Odometry::laser_sim_callback, this, std::placeholders::_1));

    // Servers
    odom_reset_srv = create_service<std_srvs::srv::Empty>(
      "odom_reset",
      std::bind(&Odometry::odom_reset_callback, this, std::placeholders::_1, std::placeholders::_2));

    // Timers
    int cycle_time = 1000.0 / loop_rate;
    int encoder_cycle_time = 1000.0 / encoder_rate;
    int path_cycle_time = 1000.0 / path_rate;
    encoder_timer = this->create_wall_timer(
      std::chrono::milliseconds(encoder_cycle_time),
      std::bind(&Odometry::encoder_timer_callback, this));
    imu_timer = this->create_wall_timer(
      std::chrono::milliseconds(cycle_time),
      std::bind(&Odometry::imu_timer_callback, this));
    path_timer = this->create_wall_timer(
      std::chrono::milliseconds(path_cycle_time),
      std::bind(&Odometry::path_timer_callback, this));
      
    // Transform broadcaster and listener
    tf_broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    tf_buffer =
      std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener =
      std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  }

private:
  // Initialize parameter variables
  int _rate;
  double raw_accel[3], accel[3], raw_gyro[3];
  double angles[3], gyro_angles[3], accel_angles[3];
  double linear_vel, linear_accel, angular_vel;
  int rate;
  double loop_rate, encoder_rate, path_rate, gear_ratio, alpha, beta, encoder_rate_sim;
  int encoder_ticks;
  uint16_t angle, angle_prev;
  double delta, wheel_diameter, chassis_speed;
  double wheel_speed_sim, wheel_speed_sim_prev, gyro_thresh, accel_thresh, vel_thresh;
  bool simulate, publish_path;
  double x_pos, y_pos, theta, theta_prev;
  nav_msgs::msg::Path path;
  geometry_msgs::msg::PoseStamped pose;

  // Initialize encoder object 
  AS5048A encoder = AS5048A("/dev/spidev0.0", SPI_MODE_1, 1000000, 8); // Adjust as necessary

  // Create ROS publishers, timers, broadcasters, etc.
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_encoder_pub;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr gyro_sub;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr accel_sub;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sim_sub;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr odom_reset_srv;
  rclcpp::TimerBase::SharedPtr encoder_timer;
  rclcpp::TimerBase::SharedPtr imu_timer;
  rclcpp::TimerBase::SharedPtr path_timer;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

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

    // Normalize rotation around vertical
    while (angles[1] > 2*3.14159265) {
      angles[1] += -2*3.1415926;
    }
    while (angles[1] < 0.) {
      angles[1] += 2*3.1415926;
    }

    // Calculate current forward velocity based on gyro angles and accel
    linear_accel = -(raw_accel[1] * sin(angles[0])) + (raw_accel[2] * cos(angles[0]));
    // vertical_accel = ((raw_accel[0] * cos(angles[0]) * sin(angles[2])) + (raw_accel[1] * cos(angles[0]) * cos(angles[2])) + (-raw_accel[2] * sin(angles[0]) * cos(angles[2])));
    if ((linear_accel < accel_thresh && (linear_accel > -accel_thresh))) {
      linear_accel = 0.0;
    }

    // Find current linear and angular velocity
    linear_vel += (linear_accel / loop_rate);
    angular_vel = raw_gyro[1];

    // Blend linear_vel with chassis speed from encoder if magnitude of 
    linear_vel = (chassis_speed * (beta)) + (linear_vel * (1. - (beta)));

    // threshold linear velocity
    if ((linear_vel < vel_thresh) && (linear_vel > -vel_thresh)) {
      linear_vel = 0.;
    }

    // Add velocities to twist message
    odom_msg.twist.twist.linear.x = linear_vel;
    odom_msg.twist.twist.angular.z = angular_vel;
    odom_msg.pose.pose.orientation.x = angles[0];
    odom_msg.pose.pose.orientation.y = angles[1];
    odom_msg.pose.pose.orientation.z = angles[2];

    // Publish odom_msg
    odom_pub->publish(odom_msg);

    // Publish base link to odom tf
    publish_odom_tf();
  }

  /// \brief The main timer callback, updates diff_drive state and publishes odom messages
  void encoder_timer_callback()
  {
    nav_msgs::msg::Odometry odom_msg;
    double wheel_speed;

    // Check if if using simulation or not
    if (simulate) {
      // If using simulation, read joint state messages to find wheel speed
      wheel_speed = wheel_speed_sim;      
    } else {
      // If not using simulation, read encoder for wheel speed
      angle = encoder.readAngle();

      int margin = 25;

      // Filtering weird encoder readings
      // if ((2*angle+margin > angle_prev) && (2*angle-margin < angle_prev)) {
      //   angle = angle_prev;
      // } else if ((0.5*angle+margin > angle_prev) && (0.5*angle-margin < angle_prev)) {
      //   angle = angle_prev;
      // } else if (((2*angle-encoder_ticks)+margin > angle_prev) && ((2*angle-encoder_ticks)-margin < angle_prev)) {
      //   angle = angle_prev;
      // } else if ((0.5*(angle+encoder_ticks)+margin > angle_prev) && (0.5*(angle+encoder_ticks)-margin < angle_prev)) {
      //   angle = angle_prev;
      // }

      // Calculate change since last reading
      delta = angle - angle_prev;
      if (abs(delta) > encoder_ticks/2) {
        if (abs(delta)==delta) {
          delta = -(encoder_ticks - abs(delta));
        } else {
          delta = (encoder_ticks - abs(delta));
        }
      }

      // RCLCPP_INFO(this->get_logger(), "delta: %f, \nangle: %d", delta, angle);
      
      if (abs(delta) < 5.) {
        delta = 0.0;
      }
      // RCLCPP_INFO(this->get_logger(), "delta: %f", delta);

      // Calculate wheel_speed
      wheel_speed = -(delta / (encoder_ticks * gear_ratio)) * encoder_rate * 6.283185307;
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
      raw_accel[1] = -msg.linear_acceleration.x;
      raw_accel[2] = -msg.linear_acceleration.y;
    }

    for (int i = 0; i < 3; i++) {
        if ((raw_accel[i] < accel_thresh && (raw_accel[i] > -accel_thresh))) {
          raw_accel[i] = 0.0;
        }
    }
  }

  /// \brief The joint_states callback function, stores the wheel speed of the rear wheels
  void joint_states_sim_callback(const sensor_msgs::msg::JointState & msg)
  {
    double speed = msg.velocity[7];
    if(abs(speed - wheel_speed_sim_prev) > 100.) {
      wheel_speed_sim = wheel_speed_sim_prev;
    } else {
      wheel_speed_sim = speed;
    }

    wheel_speed_sim_prev = wheel_speed_sim;
  }

  /// \brief Callback reseting odometry, should only be used when robot is stationary and level at start position
  void odom_reset_callback(
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

    // Reset odom->base tf
    x_pos = 0.;
    y_pos = 0.;
    theta = 0.;
    publish_odom_tf();

    // Reset map->odom tf
    // Create tf message
    geometry_msgs::msg::TransformStamped tf_map_odom_msg;
    tf_map_odom_msg.header.stamp = get_clock()->now();
    tf_map_odom_msg.header.frame_id = "odom";
    tf_map_odom_msg.child_frame_id = "base_link";
    
    // Fill out tf message
    tf_map_odom_msg.transform.translation.x = x_pos;
    tf_map_odom_msg.transform.translation.y = y_pos;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, theta);
    tf_map_odom_msg.transform.rotation.x = quaternion.x();
    tf_map_odom_msg.transform.rotation.y = quaternion.y();
    tf_map_odom_msg.transform.rotation.z = quaternion.z();
    tf_map_odom_msg.transform.rotation.w = quaternion.w();

    // Send tf_map_odom
    tf_broadcaster->sendTransform(tf_map_odom_msg);

    RCLCPP_INFO(this->get_logger(), "Odom Reset.");
  }


  /// \brief Publishes odom to base_link transform
  void publish_odom_tf()
  {
    double rad_curve, delta_theta;
    double x_new_rob, y_new_rob; 

    // Find transform in robot frame, assume robot starts at (0,0)
    if (angular_vel!=0.){
      rad_curve = linear_vel / angular_vel;
      delta_theta = angular_vel / loop_rate;
      x_new_rob = (rad_curve * sin(delta_theta));
      y_new_rob = (rad_curve * cos(delta_theta)) - rad_curve;
    } else {
      x_new_rob = linear_vel / loop_rate;
      y_new_rob = 0.;
    }

    theta += -angular_vel / loop_rate;

    // Find tf in odom frame
    x_pos += ((cos(theta_prev) * x_new_rob) - (sin(theta_prev) * y_new_rob));
    y_pos += ((sin(theta_prev) * x_new_rob) + (cos(theta_prev) * y_new_rob));

    // Create tf message
    geometry_msgs::msg::TransformStamped tf_odom_base_msg;
    tf_odom_base_msg.header.stamp = get_clock()->now();
    tf_odom_base_msg.header.frame_id = "odom";
    tf_odom_base_msg.child_frame_id = "base_link";
    
    // Fill out tf message
    tf_odom_base_msg.transform.translation.x = x_pos;
    tf_odom_base_msg.transform.translation.y = y_pos;
    tf2::Quaternion quaternion;
    quaternion.setRPY(0, 0, theta);
    tf_odom_base_msg.transform.rotation.x = quaternion.x();
    tf_odom_base_msg.transform.rotation.y = quaternion.y();
    tf_odom_base_msg.transform.rotation.z = quaternion.z();
    tf_odom_base_msg.transform.rotation.w = quaternion.w();

    // Send tf_map_odom
    tf_broadcaster->sendTransform(tf_odom_base_msg);

    theta_prev = theta;
  }

  /// \brief Read scan messages from the sim and publish them again. Fixes clock issues with slam toolbox
  void laser_sim_callback(const sensor_msgs::msg::LaserScan & msg)
  {
    sensor_msgs::msg::LaserScan scan_msg;

    // Copy data into scan message, reset timestamp, and publish
    scan_msg = msg;
    scan_msg.header.stamp = get_clock()->now();
    laser_pub->publish(scan_msg);
  }

  // /// \brief Publishes joint states for wheels
  // void publish_joint_states()
  // {
  //   // Initialize local variables and messages
  //   sensor_msgs::msg::JointState wheel_state;

  //   // Read encoder positions and time from msg and calculate angles
  //   left_encoder_ticks = msg.left_encoder;
  //   right_encoder_ticks = msg.right_encoder;
  //   time_now_sensor = (double)(msg.stamp.sec) + ((double)(msg.stamp.nanosec) * 0.000000001);
  //   angle_left_wheel = left_encoder_ticks / encoder_ticks_per_rad;      // radians
  //   angle_right_wheel = right_encoder_ticks / encoder_ticks_per_rad;    // radians

  //   // Calculate wheel speeds
  //   wheel_speed_left = ((left_encoder_ticks - left_encoder_ticks_prev) / encoder_ticks_per_rad) /
  //     (time_now_sensor - time_prev_sensor);
  //   wheel_speed_right = ((right_encoder_ticks - right_encoder_ticks_prev) / encoder_ticks_per_rad) /
  //     (time_now_sensor - time_prev_sensor);

  //   // Add headers to JointStates
  //   // Refer to Citation [3] ChatGPT
  //   wheel_state.header.stamp = get_clock()->now();

  //   // Enter information into joint state messages
  //   wheel_state.name = {"wheel_left_joint", "wheel_right_joint"};
  //   wheel_state.position = {angle_left_wheel, angle_right_wheel};
  //   wheel_state.velocity = {wheel_speed_left, wheel_speed_right};

  //   // Update time_prev_sensor and encoders prev
  //   time_prev_sensor = time_now_sensor;
  //   left_encoder_ticks_prev = left_encoder_ticks;
  //   right_encoder_ticks_prev = right_encoder_ticks;

  //   // Publish joint states for both wheels
  //   joint_states_pub->publish(wheel_state);
  // }

  /// \brief Updates the robot path with the current pose and publishes the path
  void path_timer_callback()
  {
    if (publish_path) {
      geometry_msgs::msg::TransformStamped transformStamped;
      try {
        transformStamped = tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);

        // Update ground truth red turtle path
        path.header.stamp = get_clock()->now();
        path.header.frame_id = "map";

        // Create new pose stamped
        pose.header.stamp = get_clock()->now();
        pose.header.frame_id = "map";
        pose.pose.position.x = transformStamped.transform.translation.x;
        pose.pose.position.y = transformStamped.transform.translation.y;
        pose.pose.position.z = 0.0;

        // Add rotation quaternion about Z
        pose.pose.orientation.x = transformStamped.transform.rotation.x;
        pose.pose.orientation.y = transformStamped.transform.rotation.y;
        pose.pose.orientation.z = transformStamped.transform.rotation.z;
        pose.pose.orientation.w = transformStamped.transform.rotation.w;

        // Add pose to path and publish path
        path.poses.push_back(pose);
        path_pub->publish(path);

      } catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Could not transform from map to base_link: %s", ex.what());
      }
    }
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
