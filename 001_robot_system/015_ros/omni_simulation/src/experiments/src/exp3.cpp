#include "rclcpp/rclcpp.hpp"
#include <math.h>
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;

class PDCtl : public rclcpp::Node{
public:
    PDCtl() Node("pd_ctl"){
        // subscription
        imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10, std::bind(&YawController::imu_cb, this, _1));
            
        // publisher
        yaw_publisher = this->create_publisher<std_msgs::msg::Float64>(
            "yaw_rad", 10);
    }

private:
    // convert quat to yaw radian
    double quat2yaw(double x, double y, double z, double w){
        return atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z));
    }
    void imu_cb(const sensro_msgs::msg::Imu msg){
        ouble x,y,z,w;
        x = msg.orientation.x; y = msg.orientation.y;
        z = msg.orientation.z; w = msg.orientation.w;
        yaw = quat2yaw(x,y,z,w);

        
    }
}
