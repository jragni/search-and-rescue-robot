#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

/**
 * In order to estimate the odometry in place of encoders, the cmd_velocity is integrated by time
 * to calculate the linear positions
 */
class BaseNode : public rclcpp::Node {
  public:
    BaseNode() : Node("base_node"),
      linear_velocity_x_(0.0),
      linear_velocity_y_(0.0),
      angular_velocity_z_(0.0),
      vel_dt_(0.0),
      last_vel_time_(0),
      x_pos_(0.0),
      y_pos_(0.0),
      heading_(0.0) {

      odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
      velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "cmd_vel",
        50,
        std::bind(&BaseNode::velocity_callback, this, std::placeholders::_1)
      );

      this->declare_parameter("linear_scale", 1.2);
      linear_scale_ = this->get_parameter("linear_scale").as_double();

      RCLCPP_INFO(this->get_logger(), "Starting base_node...");
    }
    
    private:
      void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // Scale the velocity message
        rclcpp::Time current_time = this->now();
        linear_velocity_x_ = msg->linear.x * linear_scale_;
        linear_velocity_y_ = msg->linear.y * linear_scale_;
        angular_velocity_z_ = msg->angular.z;

        vel_dt_ = (current_time - last_vel_time_).seconds();
        last_vel_time_ = current_time;

        double delta_heading = angular_velocity_z_ * vel_dt_; // [rad]
        double delta_x = (linear_velocity_x_ * cos(heading_) - linear_velocity_y_ * sin(heading_)) * vel_dt_; // [m]
        double delta_y = (linear_velocity_x_ * sin(heading_) + linear_velocity_y_ * cos(heading_)) * vel_dt_; // [m]

        // update the robot's position
        x_pos_ += delta_x;
        y_pos_ += delta_y;
        heading_ += delta_heading;

        tf2::Quaternion odom_quat;
        odom_quat.setRPY(0, 0, heading_);

        geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(odom_quat);

        nav_msgs::msg::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        RCLCPP_INFO(this->get_logger(), "%s", odom.header.frame_id.c_str());
        // robot's pose in x, y, and z
        odom.pose.pose.position.x = x_pos_;
        odom.pose.pose.position.y = y_pos_;
        odom.pose.pose.position.z = 0.0;

        // robot's heading in quaternion
        odom.pose.pose.orientation = msg_quat;
        odom.pose.covariance[0] = 0.001;
        odom.pose.covariance[7] = 0.001;
        odom.pose.covariance[35] = 0.001;

        // linear speed from encoders
        odom.twist.twist.linear.x = linear_velocity_x_;
        odom.twist.twist.linear.y = linear_velocity_y_;
        odom.twist.twist.linear.z = 0.0;
        odom.twist.twist.angular.x = 0.0;
        odom.twist.twist.angular.y = 0.0;

        // angular speed from encoders
        odom.twist.twist.angular.z = angular_velocity_z_;
        odom.twist.covariance[0] = 0.001;
        odom.twist.covariance[7] = 0.001;
        odom.twist.covariance[35] = 0.001;

        RCLCPP_INFO(this->get_logger(), "Odom PUBLISH");
        odom_publisher_->publish(odom);
      }

      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscriber_;

      float linear_velocity_x_;
      float linear_velocity_y_;
      float angular_velocity_z_;

      float vel_dt_;
      rclcpp::Time last_vel_time_;

      float x_pos_;
      float y_pos_;
      float heading_;

      float linear_scale_;

};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<BaseNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}