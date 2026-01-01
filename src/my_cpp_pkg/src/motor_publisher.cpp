#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MotorPublisher : public rclcpp::Node
{
public:
  MotorPublisher()
  : Node("motor_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("sensor_data", 10);
    // Publish every 60 seconds (1 minute) as requested
    timer_ = this->create_wall_timer(
      60s, std::bind(&MotorPublisher::timer_callback, this));
    RCLCPP_INFO(this->get_logger(), "Motor Publisher Node has been started. Publishing every 1 minute.");
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "True";
    // "sensor is py motor is cpp like ,,,dont overwrite nay ting in pkg file create frshly and add in setup also give how to start"
    // The request says "sensor will return true to motor". It's a bit ambiguous who sends what to whom based on "sensor is py motor is cpp".
    // Usually "sensor returns to motor" implies Sensor -> Motor.
    // BUT user said: "run subsecribe and publish like from py pkg i start listening ... it will print got response from cpp pkg same as like in cpp package as publishing"
    // AND "after 1 min it will send data to py pkg like sensor will return true to motor"
    // If Py listens and Cpp publishes, then Cpp sends to Py.
    // "sensor is py motor is cpp" + "sensor will return true to motor" contradicts "send data to py pkg".
    // Let's stick to the Plan: Cpp (MotorPublisher) -> Py (SensorSubscriber).
    // The message data will be "True" as requested in "return true to motor".
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorPublisher>());
  rclcpp::shutdown();
  return 0;
}
