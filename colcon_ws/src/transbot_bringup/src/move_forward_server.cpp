#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "transbot_msgs/srv/move_forward.hpp"

using namespace std::chrono_literals;

class MoveForwardServer : public rclcpp::Node {
  public:
    MoveForwardServer() : Node("move_forward_service_server_node") {
      pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
      srv_ = this->create_service<transbot_msgs::srv::MoveForward>(
         "move_forward",
         std::bind(
          &MoveForwardServer::move_forward_callback,
          this,
          std::placeholders::_1,
          std::placeholders::_2
        )
      );
    }

  private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
    rclcpp::Service<transbot_msgs::srv::MoveForward>::SharedPtr srv_;

    void move_forward_callback(
      const std::shared_ptr<transbot_msgs::srv::MoveForward::Request> request,
      const std::shared_ptr<transbot_msgs::srv::MoveForward::Response> response
    ) {
      auto msg = geometry_msgs::msg::Twist();
      msg.linear.x = request->velocity;
      for (int i = 0; i < request->time; i++) {
        pub_->publish(msg);
        std::this_thread::sleep_for(1000ms);
      }

      auto stop_msg = geometry_msgs::msg::Twist();
      pub_->publish(stop_msg);

      response->success = true;
    }
};


int main (int argc, char** argv) {
  rclcpp::init(argc, argv);

  std::shared_ptr<MoveForwardServer> client = std::make_shared<MoveForwardServer>();
  rclcpp::spin(client);

  rclcpp::shutdown();
  return 0;
}