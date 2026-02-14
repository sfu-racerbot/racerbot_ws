#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

class Relay : public rclcpp::Node
{
public:
    Relay() : Node("relay")
    {
        RCLCPP_INFO(this->get_logger(), "Relay node started");

        // Publisher
        publisher_ = this->create_publisher<
        ackermann_msgs::msg::AckermannDriveStamped>("drive_relay", 10);

        // Subscriber
        subscription_ = this->create_subscription<
        ackermann_msgs::msg::AckermannDriveStamped>(
            "drive",
            10,
            [this](ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg)
            {
                ackermann_msgs::msg::AckermannDriveStamped out_msg = *msg;
                out_msg.header.stamp = this->now(); // Update timestamp
                out_msg.drive.speed *= 3.0;
                out_msg.drive.steering_angle *= 3.0;
                this->publisher_->publish(out_msg);
            }
        );
    }
private:
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Relay>());
    rclcpp::shutdown();
    return 0;
}
