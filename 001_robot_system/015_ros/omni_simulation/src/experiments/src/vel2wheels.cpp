#include "rclcpp/rclcpp.hpp"
#include <math.h>
#include <eigen3/Eigen/Dense>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using Eigen::MatrixXd; using Eigen::Matrix3d;
using Eigen::Vector3d;

class Vel2Wheels : public rclcpp::Node{
public:
    Vel2Wheels() : Node("vel2wheels"){
        // subscription
        cmd_vel_subscription_ = this -> create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&Vel2Wheels::cmd_vel_cb, this, _1));
            
        // publisher
        cmd_wheel_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "cmd_wheel_vel", 10);
    }

private:
    Vector3d vel2wheels(Vector3d vel_vec){
        // set yaw angle to 0 rad
        double yaw = 0;

        // conversion matrix
        Matrix3d mat;
        mat << cos(yaw                + 1.0/2 * M_PI), cos(yaw               ), 1,
                cos(yaw + 2.0/3 * M_PI + 1.0/2 * M_PI), cos(yaw + 2.0/3 * M_PI), 1,
                cos(yaw - 2.0/3 * M_PI + 1.0/2 * M_PI), cos(yaw - 2.0/3 * M_PI), 1;        
        // velocity size for 3 wheels
        Vector3d wheel_vel_vec = mat * vel_vec;

        return wheel_vel_vec;
    }

    void cmd_vel_cb(const geometry_msgs::msg::Twist msg){
        // get velocity from key input
        double vx = msg.linear.x;
        double vy = msg.linear.y;
        double vyaw = 0;

        // convert velocity into vector and calculate velocity for 3 wheels
        Vector3d vel_vec;
        vel_vec << vx, vy, vyaw;
        Vector3d wheel_vel_vec = vel2wheels(vel_vec);
        
        // move wheels
        std_msgs::msg::Float64MultiArray cmd_wheel_vel;
        for(int i=0; i<3; i++){
            cmd_wheel_vel.data.push_back(wheel_vel_vec(i,0));
        }
        cmd_wheel_vel_publisher_->publish(cmd_wheel_vel);
    }

    // subscription
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
    // publisher
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_wheel_vel_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Vel2Wheels>());
    rclcpp::shutdown();
    return 0;
}
