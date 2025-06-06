#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "vesc_msgs/msg/throttle_board.hpp"
#include "vesc_msgs/msg/current.hpp"

enum ControlMode {
    ThrottleBoard = 0,
    Current = 1,
    Rpm = 2
};

class JoyConverterNode : public rclcpp::Node {
public:
    JoyConverterNode() : Node("joy_converter_node"), current_mode_(ThrottleBoard) {
        using std::placeholders::_1;
        
        // Joystick input subscriber
        input_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", 10,
            std::bind(&JoyConverterNode::joy_converter_callback, this, std::placeholders::_1));
        
        // control mode subscriber 
        mode_sub_ = this->create_subscription<std_msgs::msg::Int32>("control_mode", rclcpp::QoS(10), std::bind(&JoyConverterNode::mode_callback, this, _1));
            
        // three control mode publisher
        tb_pub_ = this->create_publisher<vesc_msgs::msg::ThrottleBoard>("/vesc/set_throttle_board", rclcpp::QoS(10));
        current_pub_ = this->create_publisher<vesc_msgs::msg::Current>("/vesc/set_current", rclcpp::QoS(10));
        rpm_pub_ = this->create_publisher<vesc_msgs::msg::Rpm>("/vesc/set_rpm", rclcpp::QoS(10));
    } 
        
private:
    int current_mode_;   
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr input_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
    rclcpp::Publisher<vesc_msgs::msg::ThrottleBoard>::SharedPtr tb_pub_;
    rclcpp::Publisher<vesc_msgs::msg::Current>::SharedPtr current_pub_;
    rclcpp::Publisher<vesc_msgs::msg::Rpm>::SharedPtr rpm_pub_;
    
    void joy_converter_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        switch (current_mode_) {
            case ThrottleBoard:
                auto converted_msg = vesc_msgs::msg::ThrottleBoard();
                converted_msg.throttle = msg->axes[4];
                converted_msg.board = msg->axes[0]; 
                tb_pub_->publish(converted_msg);
                
            case Current:
                auto converted_msg = vesc_msgs::msg::Current();
                converted_msg.current0 = msg->axes[1];
                converted_msg.current1 = msg->axes[4];
                current_pub_->publish(converted_msg);
                
            case Rpm:
                auto converted_msg = vesc_msgs::msg::Rpm();
                converted_msg.Rpm0 = msg->axes[1];
                converted_msg.Rpm1 = msg->axes[4];
                rpm_pub_->publish(converted_msg);
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyConverterNode>());
    rclcpp::shutdown();
    return 0;
}  
