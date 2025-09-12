#include "rclcpp/rclcpp.hpp"
#include "vehiclecontrol/msg/control.hpp" // Your custom command message
#include "feedback/msg/velocity.hpp"       // Your custom feedback message

#include <string>
#include <vector>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring> // For memcpy
#include <fcntl.h>

// --- Configuration ---
const std::string CAN_INTERFACE = "can1"; // Change if your KVASER is on can1, etc.
const int DBW_CMD_CAN_ID    = 0x120;
const int DBW_STATUS_CAN_ID = 0x121;

class DBWInterfaceNode : public rclcpp::Node
{
public:
    DBWInterfaceNode() : Node("dbw_interface_node")
    {
        // --- Initialize CAN Socket ---
        if (!initialize_can_socket()) {
            rclcpp::shutdown();
            return;
        }

        // --- Create ROS 2 Publisher and Subscriber ---
        control_subscriber_ = this->create_subscription<vehiclecontrol::msg::Control>(
            "/vehicle_control", 10,
            std::bind(&DBWInterfaceNode::control_callback, this, std::placeholders::_1));

        velocity_publisher_ = this->create_publisher<feedback::msg::Velocity>("/vehicleSpeed", 10);

        // --- Create a timer to continuously read from the CAN bus ---
        can_read_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20), // Check for messages every 20ms (50 Hz)
            std::bind(&DBWInterfaceNode::can_read_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "DBW CAN Interface Node has started successfully.");
    }

    ~DBWInterfaceNode()
    {
        if (can_socket_ != -1) {
            close(can_socket_);
        }
    }

private:
    // --- ROS 2 Members ---
    rclcpp::Subscription<vehiclecontrol::msg::Control>::SharedPtr control_subscriber_;
    rclcpp::Publisher<feedback::msg::Velocity>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr can_read_timer_;
    int can_socket_ = -1;

    // --- ROS 2 Callback for receiving commands ---
    void control_callback(const vehiclecontrol::msg::Control::SharedPtr msg)
    {
        // --- Convert ROS message to CAN frame ---
        
        // Convert throttle from float (0.0-1.0) to uint8_t (0-100)
        uint8_t throttle_percent = static_cast<uint8_t>(msg->throttle * 100.0f);
        throttle_percent = std::min(throttle_percent, (uint8_t)100); // Clamp at 100

        // Brake is already an int8, but we'll cast to uint8_t for the CAN frame
        uint8_t brake_command = (msg->brake > 0) ? 1 : 0;
        
        RCLCPP_INFO(this->get_logger(), "Sending CAN Command -> Throttle: %d%%, Brake: %d", throttle_percent, brake_command);

        // --- Prepare and send the CAN frame ---
        struct can_frame frame;
        frame.can_id = DBW_CMD_CAN_ID;
        frame.can_dlc = 2;
        frame.data[0] = throttle_percent;
        frame.data[1] = brake_command;
        // The other bytes are automatically zeroed by the kernel if dlc < 8

        if (write(can_socket_, &frame, sizeof(struct can_frame)) != sizeof(struct can_frame)) {
            RCLCPP_ERROR(this->get_logger(), "Error writing to CAN socket");
        }
    }

    // --- Main loop for reading CAN messages ---
    void can_read_loop()
    {
        struct can_frame frame;
        int nbytes = read(can_socket_, &frame, sizeof(struct can_frame));

        if (nbytes < 0) {
            // No message available, which is normal.
            return;
        }

        if (nbytes < sizeof(struct can_frame)) {
            RCLCPP_ERROR(this->get_logger(), "Incomplete CAN frame received");
            return;
        }

        // --- Process the received CAN frame ---
        if (frame.can_id == DBW_STATUS_CAN_ID && frame.can_dlc >= 4) {
            float motor_rpm_feedback;
            // Use memcpy to safely copy the bytes into a float
            std::memcpy(&motor_rpm_feedback, frame.data, sizeof(float));

            // Create a Velocity message to publish
            auto velocity_msg = feedback::msg::Velocity();
            velocity_msg.vehicle_velocity = motor_rpm_feedback; // For now, we put Motor RPM here
            // The rest are 0.0 by default from your message definition
            
            // Here you would add the logic to convert motor RPM to vehicle_velocity
            // For now, let's just publish the RPM
            velocity_publisher_->publish(velocity_msg);
        }
    }

    // --- Helper function to set up the SocketCAN interface ---
    bool initialize_can_socket()
    {
        can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (can_socket_ < 0) {
            RCLCPP_FATAL(this->get_logger(), "Error while opening CAN socket");
            return false;
        }

        struct ifreq ifr;
        std::strcpy(ifr.ifr_name, CAN_INTERFACE.c_str());
        ioctl(can_socket_, SIOCGIFINDEX, &ifr);

        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;

        // Set socket to be non-blocking
        int flags = fcntl(can_socket_, F_GETFL, 0);
        fcntl(can_socket_, F_SETFL, flags | O_NONBLOCK);

        if (bind(can_socket_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
            RCLCPP_FATAL(this->get_logger(), "Error in CAN socket bind");
            return false;
        }
        return true;
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DBWInterfaceNode>());
    rclcpp::shutdown();
    return 0;
}
