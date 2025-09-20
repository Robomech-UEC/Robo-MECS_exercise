#include "rclcpp/rclcpp.hpp"
#include <math.h>
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class PDCtl : public rclcpp::Node{
public:
    PDCtl() : Node("pd_ctl"){
        // subscription
        cmd_vel_subscription_ = this -> create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&PDCtl::cmd_vel_cb, this, _1));
            
        // publisher
        cmd_x_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "cmd_x_vel", 10);
    }

private:
    double vel_x, vel_y;

    void cmd_vel_cb(const geometry_msgs::msg::Twist msg){
        vel_x = msg.linear.x;
        vel_y = msg.linear.y;

        std_msgs::msg::Float64 cmd_x_vel;
        cmd_x_vel.data = vel_x;
        cmd_x_vel_publisher_->publish(cmd_x_vel);
    }

    // subscription
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    // publisher
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_x_vel_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PDCtl>());
    rclcpp::shutdown();
    return 0;
}
