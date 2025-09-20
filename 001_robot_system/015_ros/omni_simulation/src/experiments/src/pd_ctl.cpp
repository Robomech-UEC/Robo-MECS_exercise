#include "rclcpp/rclcpp.hpp"
#include <math.h>
#include <eigen3/Eigen/Dense>
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using Eigen::MatrixXd; using Eigen::Matrix3d;
using Eigen::Vector3d;

class PDCtl : public rclcpp::Node{
public:
    PDCtl() : Node("pd_ctl"){
        // control parameters
        declare_parameter("yaw_kp", 1.0);
        declare_parameter("yaw_kd", 1.0);
        yaw_kp   = this->get_parameter("yaw_kp").as_double();
        yaw_kd   = this->get_parameter("yaw_kd").as_double();

        auto latest_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                            .best_effort()
                            .durability_volatile();

        // subscription
        cmd_vel_subscription_ = this -> create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&PDCtl::cmd_vel_cb, this, _1));
        yaw_rad_subscription_ = this -> create_subscription<std_msgs::msg::Float64>(
            "yaw_rad", latest_qos, std::bind(&PDCtl::yaw_rad_cb, this, _1));
            
        // publisher
        cmd_wheel_vel_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "cmd_wheel_vel", 10);

        timer_ = this->create_wall_timer(
            10ms, std::bind(&PDCtl::timer_callback, this));
    }

private:
    // goal rad = 0
    double yaw_ref = 0;
    // default yaw = 0rad
    double yaw = 0;
    double yaw_vel = 0;
    double yaw_vel_max = 0.5;
    double vx, vy, vyaw;
    // control parameters
    double yaw_kp, yaw_kd;

    double err_ex = 0;
    double normalize_angle(double angle){
        while(angle > M_PI) angle -= 2.0 * M_PI;
        while(angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }
    void yaw_rad_cb(const std_msgs::msg::Float64 msg){
        yaw = msg.data;
        
        // control
        double err = yaw_ref - yaw;
        err = normalize_angle(err);
        double err_d = err - err_ex;
        yaw_vel = 1 * ( yaw_kp*err + yaw_kd*err_d );
        RCLCPP_INFO(this->get_logger(), "err: %lf, vel: %lf", err, yaw_vel);

        // handle maimam
        if(yaw_vel > 0.5){
            yaw_vel = 0.5;
        }else if(yaw_vel < -0.5){
            yaw_vel = -0.5;
        }
        
        err_ex = err;
    }

    Vector3d vel2wheels(Vector3d vel_vec){
        // conversion matrix
        Matrix3d mat;
        mat << cos(yaw                + 1.0/2 * M_PI), cos(yaw               ), 1,
               cos(yaw + 2.0/3 * M_PI + 1.0/2 * M_PI), cos(yaw + 2.0/3 * M_PI), 1,
               cos(yaw - 2.0/3 * M_PI + 1.0/2 * M_PI), cos(yaw - 2.0/3 * M_PI), 1;        
        // velocity size for 3 wheels
        Vector3d wheel_vel_vec = mat * vel_vec;
        // add pd controlled yaw velocity
        Vector3d yaw_pd_vel_vec;
        yaw_pd_vel_vec << yaw_vel, yaw_vel, yaw_vel;
        wheel_vel_vec += yaw_pd_vel_vec;

        return wheel_vel_vec;
    }

    void cmd_vel_cb(const geometry_msgs::msg::Twist msg){
        // get velocity from key input
        vx = msg.linear.x;
        vy = msg.linear.y;
        vyaw = 0;
    }

    void timer_callback(){

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
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr yaw_rad_subscription_;
    // publisher
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_wheel_vel_publisher_;
    // timer
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PDCtl>());
    rclcpp::shutdown();
    return 0;
}
