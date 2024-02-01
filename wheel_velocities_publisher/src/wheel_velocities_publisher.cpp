#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#define W_VEL 0.1

class WheelVelocitiesPublisherNode : public rclcpp::Node {
private:
  // ros objects
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher;
  rclcpp::TimerBase::SharedPtr timer_publisher;

  // member method
  void timer_publisher_callback() {
    RCLCPP_INFO(this->get_logger(), "publishing messages to /wheel_speed");

    RCLCPP_INFO(this->get_logger(), "Moving Forward");
    this->publish_wheel_speed(W_VEL, W_VEL, W_VEL, W_VEL);

    RCLCPP_INFO(this->get_logger(), "Moving Backward");
    this->publish_wheel_speed(-W_VEL, -W_VEL, -W_VEL, -W_VEL);

    RCLCPP_INFO(this->get_logger(), "Moving Leftward");
    this->publish_wheel_speed(-W_VEL, W_VEL, -W_VEL, W_VEL);

    RCLCPP_INFO(this->get_logger(), "Moving Rightward");
    this->publish_wheel_speed(W_VEL, -W_VEL, W_VEL, -W_VEL);

    RCLCPP_INFO(this->get_logger(), "Moving Clockwise");
    this->publish_wheel_speed(W_VEL, -W_VEL, -W_VEL, W_VEL);

    RCLCPP_INFO(this->get_logger(), "Moving Anti-Clockwise");
    this->publish_wheel_speed(-W_VEL, W_VEL, W_VEL, -W_VEL);

    RCLCPP_INFO(this->get_logger(), "Stop!");
    this->publish_wheel_speed(0.0, 0.0, 0.0, 0.0);

    // halt the execution
    this->timer_publisher->cancel();
  }

  void publish_wheel_speed(float w1, float w2, float w3, float w4) {
    // publishing message to the topic /wheel_speed
    auto message = std_msgs::msg::Float32MultiArray();
    message.data = {w1, w2, w3, w4};

    // logic : 3 seconds = 30 * 0.1 seconds
    int total_message = 30;
    int loop_rate = 0.1 * 1000;

    // publish message
    while (rclcpp::ok()) {
      if (total_message >= 0) {
        // publish message to topic
        this->publisher->publish(message);
        // sleep for remaining time
        total_message--;
        std::this_thread::sleep_for(std::chrono::milliseconds(loop_rate));
      } else {
        break;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

public:
  // constructor
  WheelVelocitiesPublisherNode() : Node("wheel_velocities_publisher") {
    this->publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/wheel_speed", 10);
    this->timer_publisher = this->create_wall_timer(
        std::chrono::seconds(1 / 1),
        std::bind(&WheelVelocitiesPublisherNode::timer_publisher_callback,
                  this));
  }
};

int main(int argc, char *argv[]) {
  // initialize ros, executor and node
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<WheelVelocitiesPublisherNode>();

  // add node to executor and spin
  executor.add_node(node);
  executor.spin();

  // shutdown
  rclcpp::shutdown();
  return 0;
}