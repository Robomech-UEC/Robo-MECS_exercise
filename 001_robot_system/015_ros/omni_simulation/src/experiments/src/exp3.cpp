#include "rclcpp/rclcpp.hpp"
#include <math.h>
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float64.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

class PDCtl : public rclcpp::Node{
public:
    PDCtl() : Node("pd_ctl"){
        // subscription
        imu_subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10, std::bind(&PDCtl::imu_cb, this, _1));
            
        // publisher
        yaw_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            "yaw_rad", 10);
    }

private:
    // convert quat to yaw radian
    double quat2yaw(double x, double y, double z, double w){
        return atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z));
    }
    void imu_cb(const sensor_msgs::msg::Imu msg){
        double x,y,z,w;
        x = msg.orientation.x;
        y = msg.orientation.y;
        z = msg.orientation.z;
        w = msg.orientation.w;
        double yaw = quat2yaw(x,y,z,w);

        // publish yaw as "yaw_rad"
        std_msgs::msg::Float64 yaw_rad;
        yaw_rad.data = yaw;
        yaw_publisher_->publish(yaw_rad);
    }

    // subscription
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_subscription_;
    // publisher
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr yaw_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PDCtl>());
    rclcpp::shutdown();
    return 0;
}
