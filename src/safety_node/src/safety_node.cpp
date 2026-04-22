#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <memory>
#include <cmath>
#include <algorithm>

class Safety : public rclcpp::Node
{
public:
    Safety() : rclcpp::Node("safety_node")
    {
        RCLCPP_INFO(this->get_logger(), "Safety node started");

        // Publish to drive for braking
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("drive", 1);

        // Subscribe to odometry to get our speed
        odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/ego_racecar/odom", 
            1,
            [this](nav_msgs::msg::Odometry::ConstSharedPtr msg)
            {
                drive_callback(msg);
            }
        );

        // Subscribe to laserscan for lidar measurements
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            1,
            [this](sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
            {
                scan_callback(scan_msg);
            }
        );
    }
private:
    static constexpr float TTC_THRESHOLD_ = 0.3f;
    float speed_ = 0.0; // current longitudinal speed
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;

    // Updates speed_ to our current longitudinal velocity
    void drive_callback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
    {
        speed_ = static_cast<float>(msg->twist.twist.linear.x);
    }

    // Calculates each scan's iTTC and brakes the car if below the threshold for an imminent crash
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {
        // TTC = r / {-rdot}+ (but -rdot is +rdot for us because our range rate is positive when getting closer) 
        for (size_t i = 0; i < scan_msg->ranges.size(); ++i)
        {
            // distance between vehicle and object
            float range = scan_msg->ranges[i];

            if (!std::isfinite(range)) continue; // skip if no object in proximity

            // angle between current scan and front of car
            float angle = scan_msg->angle_min + scan_msg->angle_increment * i;
            
            if (std::abs(angle) > M_PI / 2.0) continue; // only look 180 degrees forward (not full 270)

            // rate at which distance to the object changes (positive means getting closer)
            float range_rate = speed_ * std::cos(angle);

            // rate of closing on the object (separating motion set to zero)
            float closing_rate = std::max(range_rate, 0.0f);

            if (closing_rate <= 1e-3f) continue; // skip if object is not approaching

            float iTTC = range / closing_rate;
            
            if (iTTC <= TTC_THRESHOLD_)
            {
                RCLCPP_INFO(this->get_logger(), "Brake! iTTC=%.2f", iTTC);
                ackermann_msgs::msg::AckermannDriveStamped drive_msg;
                drive_msg.drive.speed = 0.0;
                drive_pub_->publish(drive_msg);
                break;
            }

            /* Variable TTC Idea
            // (unused idea from https://github.com/JC0103/F1Tenth-Lab2/blob/master/src/safety_node.cpp)
            
            double decel = 8.26; // max braking acceleration
            double reaction_time = 0.02;

            if (speed < 0) reaction_time = 0.12;

            double required_ttc = std::abs(speed) / decel + reaction_time; // instead of fixed TTC_THRESHOLD_
            */
        }
    }
};

int main (int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Safety>());
    rclcpp::shutdown();
    return 0;
}
