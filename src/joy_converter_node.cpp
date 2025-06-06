#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "vesc_msgs/msg/throttle_board.hpp"
#include "vesc_msgs/msg/current.hpp"

enum ControlMode {
    ThrottleBoard = 0,
    Current = 1,
    Speed = 2
};

class JoyConverterNode : public rclcpp::Node {
public:
    JoyConverterNode() : Node("joy_converter_node") {
        subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&JoyConverterNode::joy_converter_callback, this, std::placeholders::_1));
            
        publisher_ = this->create_publisher<vesc_msgs::msg::ThrottleBoard>("/vesc/set_throttle_board", rclcpp::QoS(10));
    }
        
private:
    void joy_converter_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        auto converted_msg = vesc_msgs::msg::ThrottleBoard();
        converted_msg.throttle = msg->axes[4];
        converted_msg.board = msg->axes[0]; 
    
        // RCLCPP_INFO(this->get_logger(), "Joystick inputs: %f, %f", converted_msg.throttle, converted_msg.board);
        
        publisher_->publish(converted_msg);
    }
    
    // void joy_converter_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    //     auto converted_msg = vesc_msgs::msg::Current();
    //     converted_msg.current = msg->axes[4];
        
        // RCLCPP_INFO(this->get_logger(), "Joystick inputs: %f, %f", converted_msg.throttle, converted_msg.board);
        
    //     publisher_->publish(converted_msg);
    // }
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription_;
    rclcpp::Publisher<vesc_msgs::msg::ThrottleBoard>::SharedPtr publisher_;
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyConverterNode>());
    rclcpp::shutdown();
    return 0;
}
