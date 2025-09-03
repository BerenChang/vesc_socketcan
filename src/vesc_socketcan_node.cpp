#include <iostream>
#include <cerrno>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <cstring>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp"
#include "vesc_msgs/msg/throttle_board.hpp"
#include "vesc_msgs/msg/current_rel.hpp"
#include "vesc_msgs/msg/rpm_rel.hpp"
#include "vesc_msgs/msg/emergency_stop.hpp"
#include "vesc_msgs/msg/can_status1_msg.hpp"
#include "vesc_msgs/msg/can_status2_msg.hpp"
#include "vesc_msgs/msg/can_status3_msg.hpp"
#include "vesc_msgs/msg/can_status4_msg.hpp"
#include "vesc_msgs/msg/can_status5_msg.hpp"
#include "datatypes.h"
#include "utils.cpp"

class VescCanNode : public rclcpp::Node {
public:
    VescCanNode() : Node("vesc_socketcan_node"), stop_listener_(false) {
        // this->declare_parameter("vesc_id", 1);
        // this->get_parameter("vesc_id", vesc_id_);
        setup_can_socket("can0");

        using std::placeholders::_1;
        current_sub_ = this->create_subscription<vesc_msgs::msg::CurrentRel>(
            "vesc/set_current_rel", 10, std::bind(&VescCanNode::set_current_cb, this, _1));
        rpm_sub_ = this->create_subscription<vesc_msgs::msg::RpmRel>(
            "vesc/set_rpm_rel", 10, std::bind(&VescCanNode::set_rpm_cb, this, _1));
        tb_sub_ = this->create_subscription<vesc_msgs::msg::ThrottleBoard>(
            "vesc/set_throttle_board", 10, std::bind(&VescCanNode::set_throttle_board_cb, this, _1));
        zero_turn_sub_ = this->create_subscription<vesc_msgs::msg::ThrottleBoard>(
            "vesc/set_zero_turn", 10, std::bind(&VescCanNode::set_zero_turn_cb, this, _1));
            
        emergency_stop_sub_ = this->create_subscription<vesc_msgs::msg::EmergencyStop>(
            "vesc/set_emergency_stop", 10, std::bind(&VescCanNode::set_emergency_stop_cb, this, _1));
            
        can_status_1_pub_ = this->create_publisher<vesc_msgs::msg::CanStatus1Msg>("/vesc/can_status_1", rclcpp::QoS(10));
        can_status_2_pub_ = this->create_publisher<vesc_msgs::msg::CanStatus2Msg>("/vesc/can_status_2", rclcpp::QoS(10));
        can_status_3_pub_ = this->create_publisher<vesc_msgs::msg::CanStatus3Msg>("/vesc/can_status_3", rclcpp::QoS(10));
        can_status_4_pub_ = this->create_publisher<vesc_msgs::msg::CanStatus4Msg>("/vesc/can_status_4", rclcpp::QoS(10));
        can_status_5_pub_ = this->create_publisher<vesc_msgs::msg::CanStatus5Msg>("/vesc/can_status_5", rclcpp::QoS(10));
        
        pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&VescCanNode::read_can, this));

        listener_thread_ = std::thread(&VescCanNode::listen_for_can_frames, this);
    }

    ~VescCanNode() {
        stop_listener_ = true;
        if (listener_thread_.joinable()) listener_thread_.join();
        if (can_socket_ >= 0) close(can_socket_);
    }

private:
    int can_socket_ = -1;
    // uint8_t vesc_id_ = 1;
    std::atomic<bool> stop_listener_;
    std::thread listener_thread_;
    rclcpp::TimerBase::SharedPtr pub_timer_;

    rclcpp::Subscription<vesc_msgs::msg::CurrentRel>::SharedPtr current_sub_;
    rclcpp::Subscription<vesc_msgs::msg::ThrottleBoard>::SharedPtr tb_sub_;
    rclcpp::Subscription<vesc_msgs::msg::ThrottleBoard>::SharedPtr zero_turn_sub_;
    rclcpp::Subscription<vesc_msgs::msg::EmergencyStop>::SharedPtr emergency_stop_sub_;
    rclcpp::Subscription<vesc_msgs::msg::RpmRel>::SharedPtr rpm_sub_;
    // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;
    
    rclcpp::Publisher<vesc_msgs::msg::CanStatus1Msg>::SharedPtr can_status_1_pub_;
    rclcpp::Publisher<vesc_msgs::msg::CanStatus2Msg>::SharedPtr can_status_2_pub_;
    rclcpp::Publisher<vesc_msgs::msg::CanStatus3Msg>::SharedPtr can_status_3_pub_;
    rclcpp::Publisher<vesc_msgs::msg::CanStatus4Msg>::SharedPtr can_status_4_pub_;
    rclcpp::Publisher<vesc_msgs::msg::CanStatus5Msg>::SharedPtr can_status_5_pub_;

    void setup_can_socket(const std::string &iface_name) {
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to create CAN socket");
            return;
        }

        struct ifreq ifr;
        // std::strncpy(ifr.ifr_name, iface_name.c_str(), IFNAMSIZ);
        std::strncpy(ifr.ifr_name, "can0", IFNAMSIZ-1);
        if (ioctl(can_socket_, SIOCGIFINDEX, &ifr) < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to get interface index");
            return;
        }

        struct sockaddr_can addr = {};
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to bind CAN socket");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "CAN socket setup on %s", iface_name.c_str());
    }
    
    void read_can() {
        struct can_frame frame;
        int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));
        if ((nbytes > 0) && (frame.can_dlc >= 8)) {
            uint32_t eid = frame.can_id & CAN_EFF_MASK;            
            uint8_t vesc_id = eid & 0xFF;
            uint8_t cmd_id = (eid >> 8) & 0xFF;
            
            // std::cout << "frame id:" << static_cast<int>()
            // std::cout << "vesc_id:" << static_cast<int>(vesc_id) << std::endl;
            // std::cout << "cmd_id:" << static_cast<int>(cmd_id) << std::endl;
            
            if (cmd_id == 10) {
                vesc_msgs::msg::CanStatus1Msg msg;
                msg.id = vesc_id;
                int32_t rpm = (frame.data[0] << 24) | (frame.data[1] << 16) | 
                              (frame.data[2] << 8) | frame.data[3];
                int16_t current = (frame.data[4] << 8) | frame.data[5];
                int16_t duty = (frame.data[6] << 8) | frame.data[7];
                
                msg.rpm = rpm;
                msg.current = current / 10.0f;
                msg.duty = duty / 1000.0f;
                
                can_status_1_pub_->publish(msg);
                
            } else if (cmd_id == 15) {
                vesc_msgs::msg::CanStatus2Msg msg;
                msg.id = vesc_id;
                int32_t amp_hours = (frame.data[0] << 24) | (frame.data[1] << 16) | 
                              (frame.data[2] << 8) | frame.data[3];
                int32_t amp_hours_charged = (frame.data[4] << 24) | (frame.data[5] << 16) | 
                              (frame.data[6] << 8) | frame.data[7];

                msg.amp_hours = amp_hours / 10000.0f;
                msg.amp_hours_charged = amp_hours_charged / 10000.0f;
                
                can_status_2_pub_->publish(msg);
                
            } else if (cmd_id == 16) {
                vesc_msgs::msg::CanStatus3Msg msg;
                msg.id = vesc_id;
                int32_t watt_hours = (frame.data[0] << 24) | (frame.data[1] << 16) | 
                              (frame.data[2] << 8) | frame.data[3];
                int32_t watt_hours_charged = (frame.data[4] << 24) | (frame.data[5] << 16) | 
                              (frame.data[6] << 8) | frame.data[7];

                msg.watt_hours = watt_hours / 10000.0f;
                msg.watt_hours_charged = watt_hours_charged / 10000.0f;
                
                can_status_3_pub_->publish(msg);
                
            } else if (cmd_id == 17) {
                vesc_msgs::msg::CanStatus4Msg msg;
                msg.id = vesc_id;
                int16_t temp_fet = (frame.data[0] << 8) | frame.data[1];
                int16_t temp_motor = (frame.data[2] << 8) | frame.data[3];
                int16_t current_in = (frame.data[4] << 8) | frame.data[5];
                int16_t pid_pos_now = (frame.data[6] << 8) | frame.data[7];

                msg.temp_fet = temp_fet / 10.0f;
                msg.temp_motor = temp_motor / 10.0f;
                msg.current_in = current_in;
                msg.pid_pos_now = pid_pos_now / 50.0f;
                
                can_status_4_pub_->publish(msg);
                
            } else if (cmd_id == 28) {
                vesc_msgs::msg::CanStatus5Msg msg;
                msg.id = vesc_id;
                int32_t tacho_value = (frame.data[0] << 24) | (frame.data[1] << 16) | 
                              (frame.data[2] << 8) | frame.data[3];
                int16_t v_in = (frame.data[4] << 8) | frame.data[5];
                msg.tacho_value = tacho_value;
                msg.v_in = v_in / 10.0f;
                
                can_status_5_pub_->publish(msg);
            }
        }
    }       

    // set current_rel
    void send_current_command(uint8_t vesc_id, 
            CAN_PACKET_ID comm_can_id, 
            float current0 ,float current1) {

        struct can_frame frame_master;
        frame_master.can_id = vesc_id | (comm_can_id << 8);
        frame_master.can_id = 0x01 | (comm_can_id << 8);
        frame_master.can_id |= CAN_EFF_FLAG;
        frame_master.can_dlc = 4;
        int32_t send_index_master = 0;
        // uint8_t buffer[8] = {0};
        
        struct can_frame frame_slave;
        frame_slave.can_id = vesc_id | (comm_can_id << 8);
        frame_slave.can_id = 0x02 | (comm_can_id << 8);
        frame_slave.can_id |= CAN_EFF_FLAG;
        frame_slave.can_dlc = 4;
        int32_t send_index_slave = 0;

        buffer_append_float32(frame_master.data, current0, 1e5, &send_index_master);
        buffer_append_float32(frame_slave.data, current1, 1e5, &send_index_slave);

        if (write(can_socket_, &frame_master, sizeof(frame_master)) != sizeof(struct can_frame)) {
            std::cerr << "Write failed: " << std::strerror(errno) << std::endl;
            return;
        }
        
        if (write(can_socket_, &frame_slave, sizeof(frame_slave)) != sizeof(struct can_frame)) {
            std::cerr << "Write failed: " << std::strerror(errno) << std::endl;
            return;
        }
    }
    
    // set_rel
    void send_rpm_command(uint8_t vesc_id, 
            CAN_PACKET_ID comm_can_id, 
            float rpm_rel0 ,float rpm_rel1) {

        struct can_frame frame_master;
        frame_master.can_id = vesc_id | (comm_can_id << 8);
        frame_master.can_id = 0x01 | (comm_can_id << 8);
        frame_master.can_id |= CAN_EFF_FLAG;
        frame_master.can_dlc = 4;
        int32_t send_index_master = 0;
        // uint8_t buffer[8] = {0};
        
        struct can_frame frame_slave;
        frame_slave.can_id = vesc_id | (comm_can_id << 8);
        frame_slave.can_id = 0x02 | (comm_can_id << 8);
        frame_slave.can_id |= CAN_EFF_FLAG;
        frame_slave.can_dlc = 4;
        int32_t send_index_slave = 0;

        buffer_append_float32(frame_master.data, rpm_rel0, 1e5, &send_index_master);
        buffer_append_float32(frame_slave.data, rpm_rel1, 1e5, &send_index_slave);

        if (write(can_socket_, &frame_master, sizeof(frame_master)) != sizeof(struct can_frame)) {
            std::cerr << "Write failed: " << std::strerror(errno) << std::endl;
            return;
        }
        
        if (write(can_socket_, &frame_slave, sizeof(frame_slave)) != sizeof(struct can_frame)) {
            std::cerr << "Write failed: " << std::strerror(errno) << std::endl;
            return;
        }
    }

    // throttle: [-1, 1], reverse [-1, 0), forward [0, 1]
    // board: [-1, 1], left [-1, 0), right (0, 1]
    void send_throttle_board_command(uint8_t vesc_id, 
            CAN_PACKET_ID comm_can_id, 
            float throttle, float board) {

        throttle += 1; // map throttle from [-1, 1] to [0, 2] for transmission
        board += 1; // map board from [-1, 1] to [0, 2] for transmission
        struct can_frame frame;
        frame.can_id = vesc_id | (comm_can_id << 8);
        frame.can_id |= CAN_EFF_FLAG;
        frame.can_dlc = 8;
        int32_t send_index = 0;
        // uint8_t buffer[8] = {0};

        buffer_append_float32(frame.data, throttle, 1e5, &send_index);
        buffer_append_float32(frame.data, board, 1e5, &send_index);
        // frame.data = buffer;

        write(can_socket_, &frame, sizeof(frame));
    }
    
    void send_emergency_stop_command(uint8_t vesc_id, CAN_PACKET_ID comm_can_id) {
        struct can_frame frame;
        frame.can_id = vesc_id | (comm_can_id << 8);
        frame.can_id |= CAN_EFF_FLAG;
        frame.can_dlc = 4;
        write(can_socket_, &frame, sizeof(frame));
    }

    void set_current_cb(const vesc_msgs::msg::CurrentRel::SharedPtr msg) {
        send_current_command(0x01, CAN_PACKET_SET_CURRENT_REL, msg->current_rel0, msg->current_rel1);
    }

    void set_throttle_board_cb(const vesc_msgs::msg::ThrottleBoard::SharedPtr msg) {
        send_throttle_board_command(0x01, CAN_SET_THROTTLE_BOARD, msg->throttle, msg->board);
    }
    
    void set_zero_turn_cb(const vesc_msgs::msg::ThrottleBoard::SharedPtr msg) {
        send_throttle_board_command(0x01, CAN_ZERO_TURN, msg->throttle, msg->board);
    }
    
    void set_emergency_stop_cb(const vesc_msgs::msg::EmergencyStop::SharedPtr msg) {
        if (msg->emergency_stop) {
            send_emergency_stop_command(0x01, CAN_EMERGENCY_STOP);
        }
    }

    // void set_duty_cb(const std_msgs::msg::Float32::SharedPtr msg) {
    //     send_vesc_command(5, static_cast<int32_t>(msg->data * 100000));  // SET_DUTY
    // }

    void set_rpm_cb(const vesc_msgs::msg::RpmRel::SharedPtr msg) {
        send_rpm_command(0x01, CAN_SET_RPM_REL, msg->rpm_rel0, msg->rpm_rel1);
    }

    void listen_for_can_frames() {
        struct can_frame frame;
        while (!stop_listener_) {
            ssize_t nbytes = read(can_socket_, &frame, sizeof(frame));
            if (nbytes > 0) {
                // RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                //                     "Received CAN ID: 0x%X, Data: %02X %02X %02X %02X %02X %02X %02X %02X",
                //                     frame.can_id,
                //                     frame.data[0], frame.data[1], frame.data[2], frame.data[3],
                //                     frame.data[4], frame.data[5], frame.data[6], frame.data[7]);
            }
        }
    }

};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  
  auto vesc_can_node = std::make_shared<VescCanNode>();
  
  // rclcpp::spin(vesc_can_node);
  
  rclcpp::Rate rate(10);
  
  while (rclcpp::ok()) {
      rclcpp::spin(vesc_can_node);
      rate.sleep();
  }

  rclcpp::shutdown();

  return 0;
}
