#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "transbot_msgs/srv/move.hpp"

using namespace std::chrono_literals;

/**
  * MoveServer
  * 
  * Moves robot at velocity [m/s] or [rad/sec] for time [seconds]
  * in either linear or angular motion.
  * 
  * Used previously for tuning odometry
  */
class MoveServer : public rclcpp::Node {
  public:
    MoveServer() : Node("move_service_server_node") {
      pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      srv_ = this->create_service<transbot_msgs::srv::Move>(
         "move",
         std::bind(
          &MoveServer::move_forward_callback,
          this,
          std::placeholders::_1,
          std::placeholders::_2
        )
      );
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Service<transbot_msgs::srv::Move>::SharedPtr srv_;

    void move_forward_callback(
      const std::shared_ptr<transbot_msgs::srv::Move::Request> request,
      const std::shared_ptr<transbot_msgs::srv::Move::Response> response
    ) {
      auto msg = geometry_msgs::msg::Twist();
      if (request->direction == "angular")  {
        msg.angular.z = request->velocity;
      } else {
        msg.linear.x = request->velocity;
      }

      pub_->publish(msg);
      std::this_thread::sleep_for(std::chrono::duration<float>(request->time));

      auto stop_msg = geometry_msgs::msg::Twist();
      pub_->publish(stop_msg);

      response->success = true;
    }
};


int main (int argc, char** argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<MoveServer> client = std::make_shared<MoveServer>();
  rclcpp::spin(client);

  rclcpp::shutdown();
  return 0;
}