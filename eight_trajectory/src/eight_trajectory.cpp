#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include <vector>

#define R 0.50
#define Lx 0.170
#define Ly 0.269

struct Twist {
  float Vx;
  float Vy;
  float Wz;
};

class EightTrajectoryNode : public rclcpp::Node {
private:
  // member variables
  double px;
  double py;
  double yaw;

  // callback groups
  rclcpp::CallbackGroup::SharedPtr callback_g1;
  rclcpp::CallbackGroup::SharedPtr callback_g2;

  // ros objects
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr publisher;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom;
  rclcpp::TimerBase::SharedPtr timer_control;

  // member method
  void subscriber_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // reading current position from /odom topic
    this->px = msg->pose.pose.position.x;
    this->py = msg->pose.pose.position.y;

    // reading current orientation from /odom topic
    double x = msg->pose.pose.orientation.x;
    double y = msg->pose.pose.orientation.y;
    double z = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;

    // convert quaternion into euler angles
    this->yaw = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    // this->yaw = (180 / M_PI) * yaw;
  }

  void timer_control_callback() {
    // waypoints for robot motion
    std::vector<std::vector<float>> waypoints;
    waypoints = {
        {0.0, 1, -1},    {0.0, 1, 1},  {0.0, 1, 1},  {1.57, 1, -1},
        {-3.14, -1, -1}, {0.0, -1, 1}, {0.0, -1, 1}, {1.57, -1, -1},
    };

    // start waypoint execution
    int counter = 1;
    for (auto waypoint : waypoints) {
      RCLCPP_INFO(this->get_logger(), "Executing Waypoint : %d", counter);
      this->move_robot(waypoint[1], waypoint[2], waypoint[0]);
      counter++;
    }

    // halt the execution
    this->timer_control->cancel();
  }

  void move_robot(float x, float y, float phi) {
    // inflate target values with current value
    float tx = x + px;
    float ty = y + py;
    float tphi = phi + yaw;

    // loop till target done
    while (rclcpp::ok()) {
      // calculate error
      float dx = tx - px;
      float dy = ty - py;
      float dphi = tphi - yaw;
      // local control structure
      if (std::fabs(dx) > 0.1 || std::fabs(dy) > 0.1) {
        auto twist = this->delta_to_velocity(dx, dy, dphi);
        this->body_to_wheel_velocity(twist.Vx, twist.Vy, twist.Wz);
      } else {
        break;
      }
    }
  }

  Twist delta_to_velocity(float dx, float dy, float dphi) {
    // struct to manage multiple returns
    Twist twist;

    // velocity of the chassis as the time derivative of the delta
    twist.Vx = (dx * std::cos(yaw) + (dy * std::sin(yaw)));
    twist.Vy = (-dx * std::sin(yaw) + (dy * std::cos(yaw)));
    twist.Wz = dphi;

    return twist;
  }

  void body_to_wheel_velocity(float Vx, float Vy, float Wz) {
    // kinematic model of the mobile robot with four mecanum wheels
    float w1 = (1 / R) * (Wz * (-(Lx / 2) - (Ly / 2)) + Vx - Vy);
    float w2 = (1 / R) * (Wz * ((Lx / 2) + (Ly / 2)) + Vx + Vy);
    float w3 = (1 / R) * (Wz * ((Lx / 2) + (Ly / 2)) + Vx - Vy);
    float w4 = (1 / R) * (Wz * (-(Lx / 2) - (Ly / 2)) + Vx + Vy);

    // publish wheel spped to the topic /wheel_speed
    this->publish_wheel_speed(w1, w2, w3, w4);
  }

  void publish_wheel_speed(float w1, float w2, float w3, float w4) {
    // publishing message to the topic /wheel_speed
    auto message = std_msgs::msg::Float32MultiArray();

    // define message
    message.data = {w1, w2, w3, w4};

    // publish message to topic
    this->publisher->publish(message);
  }

public:
  // constructor
  EightTrajectoryNode() : Node("eight_trajectory") {
    // callback groups objects
    callback_g1 =
        this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    callback_g2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // ros node options
    rclcpp::SubscriptionOptions sub_callback_g1;
    sub_callback_g1.callback_group = callback_g1;

    // ros objects
    this->subscriber_odom = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odometry/filtered", 10,
        std::bind(&EightTrajectoryNode::subscriber_odom_callback, this,
                  std::placeholders::_1),
        sub_callback_g1);
    this->publisher = this->create_publisher<std_msgs::msg::Float32MultiArray>(
        "/wheel_speed", 10);
    this->timer_control = this->create_wall_timer(
        std::chrono::seconds(1 / 1),
        std::bind(&EightTrajectoryNode::timer_control_callback, this),
        callback_g2);
  }
};

int main(int argc, char *argv[]) {
  // initialize ros, executor and node
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<EightTrajectoryNode>();

  // add node to executor and spin
  executor.add_node(node);
  executor.spin();

  // shutdown
  rclcpp::shutdown();
  return 0;
}