#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SmartMotor : public rclcpp::Node
{
public:
  SmartMotor()
  : Node("smart_motor")
  {
    // Subscriber to receive commands
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "motor_command", 10, std::bind(&SmartMotor::command_callback, this, std::placeholders::_1));

    // Publisher to send status
    publisher_ = this->create_publisher<std_msgs::msg::String>("motor_status", 10);
    
    // Timer is created but initially canceled/not running? 
    // Actually in ROS2, creating a timer starts it. We can create it and cancel it immediately, 
    // or create it inside the callback. Creating inside callback is safer for "one-shot" behavior if the interval is long.
    
    RCLCPP_INFO(this->get_logger(), "Smart Motor Node Started. Waiting for 'Start Motor' command...");
  }

private:
  void command_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    if (msg->data == "Start Motor") {
      RCLCPP_INFO(this->get_logger(), "Received Command: 'Start Motor'. Running task for 60 seconds...");
      
      // Start a one-shot timer for 60 seconds
      // We use a unique ptr to reset/create new timer each time
      timer_ = this->create_wall_timer(
        60s, std::bind(&SmartMotor::timer_callback, this));
    }
  }

  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Completed";
    
    RCLCPP_INFO(this->get_logger(), "Task Finished. Sending: 'Completed' to Sensor.");
    publisher_->publish(message);
    
    // Stop the timer so it doesn't fire again automatically
    timer_->cancel();
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SmartMotor>());
  rclcpp::shutdown();
  return 0;
}
