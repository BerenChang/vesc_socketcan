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
    JoyConverterNode() : Node("joy_converter_node"), control_mode_(ThrottleBoard), button_press_time_(0), last_button_state_(false) {
        // using std::placeholders::_1;
        
        // parameters
        // this->declare_parameter<int>("mode_change_button", ThrottleBoard);
        // this->declare_parameter<int>("num_modes", 3);
        // this->declare_parameter<double>("hold_duration", 1.0);
        
        // get parameters
        // mode_change_button_ = this->get_parameter("mode_change_button").as_int();
        // num_modes_ = this->get_parameter("num_modes").as_int();
        // hold_duration_ = this->get_parameter("hold_duration").as_double();

        
        // Joystick input subscriber
        input_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy", rclcpp::QoS(10),
            std::bind(&JoyConverterNode::joy_converter_callback, this, std::placeholders::_1));
        
        // control mode subscriber 
        mode_sub_ = this->create_subscription<std_msgs::msg::Int32>("control_mode", rclcpp::QoS(10), std::bind(&JoyConverterNode::mode_callback, this, _1));
            
        // three control mode publisher
        tb_pub_ = this->create_publisher<vesc_msgs::msg::ThrottleBoard>("/vesc/set_throttle_board", rclcpp::QoS(10));
        current_pub_ = this->create_publisher<vesc_msgs::msg::Current>("/vesc/set_current", rclcpp::QoS(10));
        rpm_pub_ = this->create_publisher<vesc_msgs::msg::Rpm>("/vesc/set_rpm", rclcpp::QoS(10));
        
        // timer for checking button hold duration
        timer_ = this->create_wall_timer(100ms, std::bind(&JoyConverterNode::timer_callback, this));
    }

private:
    int control_mode_;
    int num_modes_;
    float mode_change_button_;
    double hold_duration_;
    double button_pressed_time_;
    bool last_button_state_;
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr input_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
    rclcpp::Publisher<vesc_msgs::msg::ThrottleBoard>::SharedPtr tb_pub_;
    rclcpp::Publisher<vesc_msgs::msg::Current>::SharedPtr current_pub_;
    rclcpp::Publisher<vesc_msgs::msg::Rpm>::SharedPtr rpm_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    void joy_converter_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    
        mode_change_button_ = msg->buttons[0]; // change to a different button later
    
        // Check if mode change button is pressed
        if (mode_change_button_ != 0) {
            bool current_button_state = (msg->buttons[0] == 1);

            // Record when button was first pressed
            if (current_button_state && !last_button_state_) {
                button_pressed_time_ = this->now().seconds();
            }

            last_button_state_ = current_button_state;
        }
    
        switch (control_mode_) {
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
                converted_msg.rpm0 = msg->axes[1];
                converted_msg.rpm1 = msg->axes[4];
                rpm_pub_->publish(converted_msg);
        }
    }
    
    void timer_callback() {
        if (last_button_state_ && (this->now().seconds() - button_pressed_time_ >= hold_duration_)) {
            control_mode_ = (control_mode_ + 1) % num_modes_;
            button_pressed_time_ = 0;
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyConverterNode>());
    rclcpp::shutdown();
    return 0;
}  
