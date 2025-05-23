#include "rclcpp/rclcpp.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
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
      last_vel_time_(this->now()),
      x_pos_(0.0),
      y_pos_(0.0),
      heading_(0.0) {

      odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("odom_raw", 50);
      velocity_subscriber_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/transbot/get_vel",
        50,
        std::bind(&BaseNode::odom_callback, this, std::placeholders::_1)
      );


      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

      RCLCPP_INFO(this->get_logger(), "Starting base_node...");
    }
    
    private:
      void odom_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        linear_velocity_x_ = msg->twist.linear.x;
        linear_velocity_y_ = 0.0;
        angular_velocity_z_ = msg->twist.angular.z;
        rclcpp::Time current_time = msg->header.stamp;
        vel_dt_ = (current_time - last_vel_time_).seconds();
        
        if (vel_dt_ > 0.0) {
            double delta_heading = angular_velocity_z_ * vel_dt_; // [rad]
            double delta_x = (linear_velocity_x_ * cos(heading_)) * vel_dt_; // [m]
            double delta_y = (linear_velocity_x_ * sin(heading_)) * vel_dt_; // [m]

            // update the robot's position
            x_pos_ += delta_x;
            y_pos_ += delta_y;
            heading_ += delta_heading;

            tf2::Quaternion odom_quat;
            odom_quat.setRPY(0, 0, heading_);

            // Convert quaternion directly without redundant assignments
            geometry_msgs::msg::Quaternion msg_quat = tf2::toMsg(odom_quat);

            nav_msgs::msg::Odometry odom;
            odom.header.stamp = current_time;
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_footprint";

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
            odom.twist.covariance[0] = 0.0001;
            odom.twist.covariance[7] = 0.0001;
            odom.twist.covariance[35] = 0.0001;

            odom_publisher_->publish(odom);

            // Handle transform publisher
            geometry_msgs::msg::TransformStamped t;

            t.header.stamp = current_time;
            t.header.frame_id = "odom";
            t.child_frame_id = "base_footprint";

            t.transform.translation.x = x_pos_;
            t.transform.translation.y = y_pos_;
            t.transform.translation.z = 0;
            t.transform.rotation.x = odom_quat.x();
            t.transform.rotation.y = odom_quat.y();
            t.transform.rotation.z = odom_quat.z();
            t.transform.rotation.w = odom_quat.w();
            tf_broadcaster_->sendTransform(t);
        }
        
        last_vel_time_ = current_time;
      }

      rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
      rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_subscriber_;
      std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

      float linear_velocity_x_;
      float linear_velocity_y_;
      float angular_velocity_z_;

      float vel_dt_;
      rclcpp::Time last_vel_time_;

      float x_pos_;
      float y_pos_;
      float heading_;

};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<BaseNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}