#include "rclcpp/rclcpp.hpp"
 
class MyCustomNode : public rclcpp::Node // MODIFY NAME
{
public:
    MyCustomNode() : Node("test_node_cpp_a") // MODIFY NAME
    {
        RCLCPP_INFO(this->get_logger(), "Hello ROS2 from cpp node");
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&MyCustomNode::timer_callback, this));
    }
 
private:
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Hello ROS2 from cpp node");
    }
    rclcpp::TimerBase::SharedPtr timer_;
};
 
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyCustomNode>(); // MODIFY NAME
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
