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
#include "std_msgs/msg/float32.hpp"
#include "vesc_msgs/msg/throttle_board.hpp"
#include "vesc_msgs/msg/current.hpp"
#include "datatypes.h"
#include "utils.cpp"

class VescCanNode2 : public rclcpp::Node {
public:
    VescCanNode2() : Node("vesc_socketcan_node2"), stop_listener_(false) {
        // this->declare_parameter("vesc_id", 1);
        // this->get_parameter("vesc_id", vesc_id_);
        setup_can_socket("can1");

        using std::placeholders::_1;
        current_sub_ = this->create_subscription<vesc_msgs::msg::Current>(
            "vesc/set_current", 10, std::bind(&VescCanNode2::set_current_cb, this, _1));
        // duty_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        //     "vesc/set_duty", 10, std::bind(&VescCanNode::set_duty_cb, this, _1));
        // rpm_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        //     "vesc/set_rpm", 10, std::bind(&VescCanNode::set_rpm_cb, this, _1));
        tb_sub_ = this->create_subscription<vesc_msgs::msg::ThrottleBoard>(
            "vesc/set_throttle_board", 10, std::bind(&VescCanNode2::set_throttle_board_cb, this, _1));

        listener_thread_ = std::thread(&VescCanNode2::listen_for_can_frames, this);
    }

    ~VescCanNode2() {
        stop_listener_ = true;
        if (listener_thread_.joinable()) listener_thread_.join();
        if (can_socket_ >= 0) close(can_socket_);
    }

private:
    int can_socket_ = -1;
    // uint8_t vesc_id_ = 1;
    std::atomic<bool> stop_listener_;
    std::thread listener_thread_;

    rclcpp::Subscription<vesc_msgs::msg::Current>::SharedPtr current_sub_;
    rclcpp::Subscription<vesc_msgs::msg::ThrottleBoard>::SharedPtr tb_sub_;
    // rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr duty_sub_;
    // rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr rpm_sub_;
    // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub_;

    void setup_can_socket(const std::string &iface_name) {
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Failed to create CAN socket");
            return;
        }

        struct ifreq ifr;
        // std::strncpy(ifr.ifr_name, iface_name.c_str(), IFNAMSIZ);
        std::strncpy(ifr.ifr_name, "can1", IFNAMSIZ-1);
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

    void send_current_command(uint8_t vesc_id, 
            CAN_PACKET_ID comm_can_id, 
            float current) {

        struct can_frame frame;
        frame.can_id = vesc_id | (comm_can_id << 8);
        frame.can_id = 0x12 | (comm_can_id << 8);
        frame.can_id |= CAN_EFF_FLAG;
        frame.can_dlc = 4;
        int32_t send_index = 0;
        // uint8_t buffer[8] = {0};

        buffer_append_float32(frame.data, current, 1e3, &send_index);
        // frame.data = buffer;

        // std::cout << "frame data: ";
        // for (int i = 0; i < 4; ++i) {
        //     std::cout << static_cast<int>(frame.data[i]) << " ";
        // }
        // std::cout << std::endl;
        
        // std::cout << "sizeof frame: " << static_cast<int>(sizeof(frame)) << std::endl;
        // std::cout << "sizeof can_frame: " << static_cast<int>(sizeof(can_frame)) << std::endl;
        // std::cout << "sizeof write:" << write(can_socket_, &frame, sizeof(frame)) << std::endl;
        
        if (write(can_socket_, &frame, sizeof(frame)) != sizeof(struct can_frame)) {
            // RCLCPP_FATAL(this->get_logger(), "CAN socket write error");
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

    void set_current_cb(const vesc_msgs::msg::Current::SharedPtr msg) {
        send_current_command(static_cast<uint8_t>(18), CAN_PACKET_SET_CURRENT, msg->current);
    }

    void set_throttle_board_cb(const vesc_msgs::msg::ThrottleBoard::SharedPtr msg) {
        send_throttle_board_command(0x12, CAN_SET_THROTTLE_BOARD, msg->throttle, msg->board);
    }

    // void set_duty_cb(const std_msgs::msg::Float32::SharedPtr msg) {
    //     send_vesc_command(5, static_cast<int32_t>(msg->data * 100000));  // SET_DUTY
    // }

    // void set_rpm_cb(const std_msgs::msg::Float32::SharedPtr msg) {
    //     send_vesc_command(3, static_cast<int32_t>(msg->data));  // SET_RPM
    // }

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

}; // end of socketcan node

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<VescCanNode2>());

  // RCLCPP_INFO(VescCanNode->get_logger(), "Vesc CanBus Node starting");

  rclcpp::shutdown();

  return 0;
}
