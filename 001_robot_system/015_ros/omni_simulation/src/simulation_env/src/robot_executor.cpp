#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/float64.hpp"

#define MAX_FOOT_RAD_S 10.5
#define SERVO_COUNT 1

using namespace std::chrono_literals;
using std::placeholders::_1;

class RobotExecutor : public rclcpp::Node {
public:
    RobotExecutor() : Node("robot_executor") {
        cmd_wheel_subscription = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "cmd_wheel_vel", 10, std::bind(&RobotExecutor::cmd_wheel_vel_cb, this, _1));
        wheel_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "wheel_controller/commands", 10);

        cmd_cam_servo_rad_subscription = this->create_subscription<std_msgs::msg::Float64>(
            "cmd_cam_servo_rad", 10, std::bind(&RobotExecutor::cmd_cam_servo_rad_cb, this, _1));
        
        servo_rad_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "servo_controller/commands", 10);

        timer_ = this->create_wall_timer(10ms, std::bind(&RobotExecutor::publish_cmd_servo_rad, this));
    }

private:
    void cmd_wheel_vel_cb(const std_msgs::msg::Float64MultiArray msg){
        std_msgs::msg::Float64MultiArray wheel_commands;
        for(int i=0; i<3; i++){
            // rpm to rad/s
            double command = msg.data[i] * 2*M_PI;
            // change direction
            command *= (-1 * MAX_FOOT_RAD_S);
            wheel_commands.data.push_back(command);
        }
        wheel_vel_publisher_->publish(wheel_commands);
    }

    std::array<double, SERVO_COUNT> servo_rad = {0};

    void cmd_cam_servo_rad_cb(const std_msgs::msg::Float64 msg){
        servo_rad[0] = msg.data * -1;
    }

    void publish_cmd_servo_rad(){
        std_msgs::msg::Float64MultiArray servo_rad_command;
        for(int i=0; i<SERVO_COUNT; i++){
            servo_rad_command.data.push_back(servo_rad[i]);
        }
        servo_rad_publisher_->publish(servo_rad_command);
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_wheel_subscription;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr wheel_vel_publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_cam_servo_rad_subscription;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr servo_rad_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotExecutor>());
    rclcpp::shutdown();
    return 0;
}
