#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"

using namespace std::chrono_literals;

class Talker : public rclcpp::Node
{
public:
    Talker() : Node("talker")
    {
        // Declare parameters with default values
        this->declare_parameter<double>("v", 0.0); // speed
        this->declare_parameter<double>("d", 0.0); // steering angle

        // Log startup message with initial parameters
        double v = this->get_parameter("v").as_double();
        double d = this->get_parameter("d").as_double();
        RCLCPP_INFO(this->get_logger(), "Talker node started with v=%.2f, d=%.2f", v, d);

        // Publisher
        publisher_ = this->create_publisher<
        ackermann_msgs::msg::AckermannDriveStamped>("drive", 10);

        // Timer Callback
        auto timer_callback = 
            [this]() -> void {
                double v = this->get_parameter("v").as_double();
                double d = this->get_parameter("d").as_double();

                ackermann_msgs::msg::AckermannDriveStamped msg;
                
                msg.header.stamp = this->now();
                msg.header.frame_id = "base_link";

                msg.drive.speed = v;
                msg.drive.steering_angle = d;
                this->publisher_->publish(msg);
            };
        
        timer_ = this->create_wall_timer(10ms, timer_callback); // Timer at 10 Hz
    }
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Talker>());
    rclcpp::shutdown();
    return 0;
}
