#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/header.hpp"

#include "imu_calib/accel_calib.h"

using namespace imu_calib;
class ApplyCalib : public rclcpp::Node {
  public:
    ApplyCalib() : Node("apply_calib"),
    gyro_sample_count_(0),
    gyro_bias_x_(0.0),
    gyro_bias_y_(0.0),
    gyro_bias_z_(0.0) {

      std::string calib_file = this->declare_parameter<std::string>(
        "calib_file",
        "imu_calib.yaml"
      );

      if (!calib_.loadCalib(calib_file) || !calib_.calibReady()) {
        RCLCPP_FATAL(this->get_logger(), "Calibration could not be loaded");
        rclcpp::shutdown();
      }

      calibrate_gyros_ = this->declare_parameter<bool>("calibrate_gyros", true);
      gyro_calib_samples_ = this->declare_parameter<int>("gyro_calib_samples", 100);

      int queue_size = this->declare_parameter<int>("queue_size", 5);

      raw_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/transbot/imu",
        queue_size,
        std::bind(&ApplyCalib::raw_imu_callback, this, std::placeholders::_1)
      );

      corrected_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu/data_raw", queue_size);
      
      RCLCPP_INFO(this->get_logger(), "apply_calib node running...");
    }

  private:
    AccelCalib calib_;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr raw_sub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr corrected_pub_;

    void raw_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
      if (calibrate_gyros_) {
        RCLCPP_INFO_ONCE(this->get_logger(), "Calibrating gyros, do not move the IMU!");
        gyro_sample_count_++;
        gyro_bias_x_ = ((gyro_sample_count_ - 1) * gyro_bias_x_ + msg->angular_velocity.x) / gyro_sample_count_;
        gyro_bias_y_ = ((gyro_sample_count_ - 1) * gyro_bias_y_ + msg->angular_velocity.y) / gyro_sample_count_;
        gyro_bias_z_ = ((gyro_sample_count_ - 1) * gyro_bias_z_ + msg->angular_velocity.z) / gyro_sample_count_;

        if (gyro_sample_count_ >= gyro_calib_samples_) {
          RCLCPP_INFO(
            this->get_logger(),
            "Gyro calibration complete! [%.3f,%.3f,%.3f]",
            gyro_bias_x_,
            gyro_bias_y_,
            gyro_bias_z_
          );
          calibrate_gyros_ = false;
        }
        return;
      }
      sensor_msgs::msg::Imu corrected = *msg;

      calib_.applyCalib(
        msg->linear_acceleration.x,
        msg->linear_acceleration.y,
        msg->linear_acceleration.z,
        &corrected.linear_acceleration.x,
        &corrected.linear_acceleration.y,
        &corrected.linear_acceleration.z
      );

      corrected.angular_velocity.x = msg->angular_velocity.x - gyro_bias_x_;
      corrected.angular_velocity.y = msg->angular_velocity.y - gyro_bias_y_;
      corrected.angular_velocity.z = msg->angular_velocity.z - gyro_bias_z_;

      corrected.header.stamp = this->get_clock()->now();
      corrected.header.frame_id = "imu_link";

      corrected_pub_->publish(corrected);
    }

    bool calibrate_gyros_;
    int gyro_calib_samples_;
    int gyro_sample_count_;

    double gyro_bias_x_;
    double gyro_bias_y_;
    double gyro_bias_z_;

};

int main (int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ApplyCalib>());
  rclcpp::shutdown();

  return 0;
}