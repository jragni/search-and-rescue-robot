#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "imu_calib/accel_calib.h"

#include <string>
#include <vector>
#include <queue>

using namespace imu_calib;

class DoCalib : public rclcpp::Node {
  public:
    DoCalib() : Node("do_calib"), state_(START) {
      measurements_per_orientation_ = this->declare_parameter<int>("measurements", 500);
      reference_acceleration_ = this->declare_parameter<int>("reference_acceleration", 9.808665);
      output_file_ = this->declare_parameter<std::string>("output_file", "imu_calib.yaml");
      RCLCPP_INFO(this->get_logger(), "do_calib node starting...");

      orientations_.push(AccelCalib::XPOS);
      orientations_.push(AccelCalib::XNEG);
      orientations_.push(AccelCalib::YPOS);
      orientations_.push(AccelCalib::YNEG);
      orientations_.push(AccelCalib::ZPOS);
      orientations_.push(AccelCalib::ZNEG);

      orientation_labels_[AccelCalib::XPOS] = "X+";
      orientation_labels_[AccelCalib::XNEG] = "X-";
      orientation_labels_[AccelCalib::YPOS] = "Y+";
      orientation_labels_[AccelCalib::YNEG] = "Y-";
      orientation_labels_[AccelCalib::ZPOS] = "Z+";
      orientation_labels_[AccelCalib::ZNEG] = "Z-";

      imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "do_calib",
        1, 
        std::bind(&DoCalib::imu_callback, this, std::placeholders::_1)
      );

    }

    bool running() {
      return state_ != DONE;
    }

  private:
    enum DoCalibState { START, SWITCHING, RECEIVING, COMPUTING, DONE };

    AccelCalib calib_;
    DoCalibState state_;
    int measurements_per_orientation_;
    int measurements_received_;

    double reference_acceleration_;
    std::string output_file_;

    std::queue<AccelCalib::Orientation> orientations_;
    AccelCalib::Orientation current_orientation_;

    std::string orientation_labels_[6];

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
      bool accepted;
      switch(state_) {
        case START:
          calib_.beginCalib(measurements_per_orientation_, reference_acceleration_);
          state_ = SWITCHING;
          break;
        case SWITCHING:
          if (orientations_.empty()) {
            state_ = COMPUTING;
          } else {
            current_orientation_ = orientations_.front();

            orientations_.pop();
            measurements_received_ = 0;

            std::cout << "Orient IMU with " << orientation_labels_[current_orientation_] << " axis up and press Enter";
            std::cin.get();
            std::cout << "Recording measurements...";
            state_ = RECEIVING;
          }
          break;
        case RECEIVING:
          accepted = calib_.addMeasurement(
            current_orientation_,
            msg->linear_acceleration.x,
            msg->linear_acceleration.y,
            msg->linear_acceleration.z
          );

          measurements_received_ += accepted ? 1 : 0;

          if (measurements_received_ >= measurements_per_orientation_) {
            std::cout << "Done." << std::endl;
          }

          state_ = SWITCHING;
          break;
        case COMPUTING:
          std::cout << "Computing calibration parameters...";
          if (calib_.computeCalib()) {
            std::cout << " Success!" << std:: endl;

            std::cout << "Saving calibration file...";
            if (calib_.saveCalib(output_file_)) {
              std::cout << " Success!" << std::endl;
            } else {
              std::cout << " Failed." << std::endl;
            }
          } else {
            std::cout << " Failed.";
            RCLCPP_ERROR(this->get_logger(), "Calibration Failed!!!");
          }
          state_ = DONE;
          break;
        case DONE:
          break;
      }
    }

};

int main (int argc, char** argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<DoCalib> node = std::make_shared<DoCalib>();

  while (rclcpp::ok() && node->running()) {
    rclcpp::spin_some(node);
  }

  return 0;
}