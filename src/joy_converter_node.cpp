#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "vesc_msgs/msg/throttle_board.hpp"
#include "vesc_msgs/msg/current.hpp"



class JoyConverterNode : public rclcpp::Node {
public: 
    enum ControlMode {
        ThrottleBoard = 0,
        Current = 1,
        Speed = 2
    };

    enum ButtonState {
        ButtonNotPressed = 0,
        TBButton = 1,
        CurrentButton = 2,
        SpeedButton = 3
    };

public:
    JoyConverterNode() 
        : Node("joy_converter_node"), control_mode_(ThrottleBoard), now_button_state_(ButtonNotPressed), last_button_state_(ButtonNotPressed), 
        num_modes_(3), tb_button_(0), current_button_(0), speed_button_(0), hold_duration_(1), button_pressed_time_(0) {
        // using std::placeholders::_1;
        
        // parameters
        // this->declare_parameter<int>("mode_change_button", ThrottleBoard);
        // this->declare_parameter<int>("num_modes", 3);
        // this->declare_parameter<double>("hold_duration", 1.0);
        
        // get parameters
        // tb_button_ = this->get_parameter("mode_change_button").as_int();
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
    enum ControlMode control_mode_;
    enum ButtonState now_button_state_;
    enum ButtonState last_button_state_;
    int num_modes_;
    float tb_button_;
    float current_button_;
    float speed_button_;
    double hold_duration_;
    double button_pressed_time_;
    
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr input_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr mode_sub_;
    rclcpp::Publisher<vesc_msgs::msg::ThrottleBoard>::SharedPtr tb_pub_;
    rclcpp::Publisher<vesc_msgs::msg::Current>::SharedPtr current_pub_;
    rclcpp::Publisher<vesc_msgs::msg::Rpm>::SharedPtr rpm_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    void joy_converter_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    
        tb_button_ = msg->buttons[0]; // change to a different button later
        current_button_ = msg->buttons[1]; // change to a different button later
        speed_button_ = msg->buttons[2]; // change to a different button later
    
        // Check if mode-change button is pressed
        if (tb_button_ != 0) { // #1 priority 
            now_button_state_ = TBButton;

            // Record when button was first pressed
            if (now_button_state_ != last_button_state_) {
                button_pressed_time_ = this->now().seconds();
                last_button_state_ = now_button_state_;
            }

        } else if (current_button_ != 0) { // #2 priority 
            now_button_state_ = CurrentButton;

            // Record when button was first pressed
            if (now_button_state_ != last_button_state_) {
                button_pressed_time_ = this->now().seconds();
                last_button_state_ = now_button_state_;
            }

        } else if (speed_button_ != 0) { // #3 priority 
            now_button_state_ = SpeedButton;

            // Record when button was first pressed
            if (now_button_state_ != last_button_state_) {
                button_pressed_time_ = this->now().seconds();
                last_button_state_ = now_button_state_;
            }
        }

        // Check if mode-change button is released
        if ((tb_button_ == 0) && (last_button_state_ == TBButton)) { // TB button is released
            now_button_state_ = ButtonNotPressed;
            button_pressed_time_ = 0;
        } else if ((current_button_ == 0) && (last_button_state_ == CurrentButton)) { // Current button is released
            now_button_state_ = ButtonNotPressed;
            button_pressed_time_ = 0;
        } else if ((speed_button_ == 0) && (last_button_state_ == SpeedButton)) { // Speed button is released
            now_button_state_ = ButtonNotPressed;
            button_pressed_time_ = 0;
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
                
            case Speed:
                auto converted_msg = vesc_msgs::msg::Rpm();
                converted_msg.rpm0 = msg->axes[1];
                converted_msg.rpm1 = msg->axes[4];
                rpm_pub_->publish(converted_msg);
        }
    }
    
    void timer_callback() {
        if (((this->now().seconds() - button_pressed_time_) >= hold_duration_)) {
            if (last_button_state_ == TBButton) {
                control_mode_ = ThrottleBoard;
                button_pressed_time_ = 0;
            } else if (last_button_state_ == CurrentButton) {
                control_mode_ = Current;
                button_pressed_time_ = 0;
            } else if (last_button_state_ == SpeedButton) {
                control_mode_ = Speed;
                button_pressed_time_ = 0;
            }
            
        }
    }
};

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoyConverterNode>());
    rclcpp::shutdown();
    return 0;
}  
