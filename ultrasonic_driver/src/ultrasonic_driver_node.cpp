#include "rclcpp/rclcpp.hpp"
#include "libHCSR04/libHCSR04.h" // maybe with a path
#include <memory>
#include <map>
#include <string>
#include <vector>
#include "sensor_msgs/msg/range.hpp"

using namespace std::chrono_literals;

class UltrasonicHCSR04Wrapper :  public rclcpp::Node
{
public:
    UltrasonicHCSR04Wrapper() : Node("ultrasonic_driver"), count_(0)
    {
        int trigger_pin;
        int echo_pin;

        this->declare_parameter("minimum_range", 2);
        this->declare_parameter("maximum_range", 400);
        this->declare_parameter("field_of_view", 15.0);
        this->declare_parameter("trigger_pin", 25);
        this->declare_parameter("echo_pin", 24);

        this->get_parameter("minimum_range", min_range_);
        this->get_parameter("maximum_range", max_range_);
        this->get_parameter("field_of_view", fov_);
        this->get_parameter("trigger_pin", trigger_pin);
        this->get_parameter("echo_pin", echo_pin);


        RCLCPP_INFO(this->get_logger(), "Ultrasonic driver is now started");
        ultrasonic_.reset(new HCSR04(trigger_pin, echo_pin));

        current_distance_publisher_ = this->create_publisher<sensor_msgs::msg::Range>("/distance", 10);
        current_velocity_publisher_ = this->create_publisher<sensor_msgs::msg::Range>("/relative_velocity", 10);

        current_distance_timer_ = this->create_wall_timer(500ms, std::bind(&UltrasonicHCSR04Wrapper::publishCurrentDistance, this));
        current_velocity_timer_ = this->create_wall_timer(500ms, std::bind(&UltrasonicHCSR04Wrapper::publishCurrentVelocity, this));

    }

    void publishCurrentDistance() 
    {
        double distance = ultrasonic_->distance(1000000);

        auto msg = sensor_msgs::msg::Range();

        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";
        msg.radiation_type = 0;
        msg.field_of_view = fov_ * 0.0174532925199433;
        msg.min_range = min_range_;
        msg.max_range = max_range_;

        distance = (distance < min_range_)? min_range_ : distance;
        distance = (distance > max_range_)? max_range_ : distance;

        msg.range = distance;

        current_distance_publisher_->publish(msg);
    }

    void publishCurrentVelocity()
    {
        double speed = ultrasonic_->speed(1000000);

        auto msg = sensor_msgs::msg::Range();

        msg.header.stamp = this->now();
        msg.header.frame_id = "base_link";
        msg.radiation_type = 0;
        msg.field_of_view = fov_ * 0.0174532925199433;
        msg.min_range = min_range_;
        msg.max_range = max_range_;

        msg.range = speed;

        current_velocity_publisher_->publish(msg);
    }

private:
    std::unique_ptr<HCSR04> ultrasonic_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr current_distance_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr current_velocity_publisher_;

    rclcpp::TimerBase::SharedPtr current_distance_timer_;
    rclcpp::TimerBase::SharedPtr current_velocity_timer_;

    int min_range_;
    int max_range_;
    double fov_;
    size_t count_;
};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UltrasonicHCSR04Wrapper>()); 
    rclcpp::shutdown();
    return 0;
}