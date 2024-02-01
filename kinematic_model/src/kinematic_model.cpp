#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#define R 0.50
#define Lx 0.170
#define Ly 0.269

class KinematicModelNode : public rclcpp::Node {
private:
  // ros objects
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr subscriber;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_cmd_vel;

  // member method
  void subscriber_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
    // define message
    auto message = geometry_msgs::msg::Twist();

    // wheels angular velocities
    float w1 = msg->data[0];
    float w2 = msg->data[1];
    float w3 = msg->data[2];
    float w4 = msg->data[3];

    // robot linear and angular velocity
    message.linear.x = (w1 + w2 + w3 + w4) * (R / 4);
    message.linear.y = (-w1 + w2 - w3 + w4) * (R / 4);
    message.angular.z = (-w1 + w2 + w3 - w4) * (R / (2 * (Lx + Ly)));

    // publish velocity
    this->publisher_cmd_vel->publish(message);
  }

public:
  // constructor
  KinematicModelNode() : Node("kinematic_model") {
    this->subscriber =
        this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/wheel_speed", 10,
            std::bind(&KinematicModelNode::subscriber_callback, this,
                      std::placeholders::_1));
    this->publisher_cmd_vel =
        this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  }
};

int main(int argc, char *argv[]) {
  // initialize ros, executor and node
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<KinematicModelNode>();

  // add node to executor and spin
  executor.add_node(node);
  executor.spin();

  // shutdown
  rclcpp::shutdown();
  return 0;
}