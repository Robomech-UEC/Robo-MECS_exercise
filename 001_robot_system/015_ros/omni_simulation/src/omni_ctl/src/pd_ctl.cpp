#include "rclcpp/rclcpp.hpp"
#include <math.h>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_srvs/srv/empty.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class PDCtl : public rclcpp::Node{
public:
    PDCtl() Node("pd_ctl"){
        declare_parameter("yaw_vmax", 1.0);
        declare_parameter("yaw_kp", 1.0);
        declare_parameter("yaw_kd", 1.0);

        yaw_vmax = this->get_parameter("yaw_vmax").as_double();
        yaw_kp   = this->get_parameter("yaw_kp").as_double();
        yaw_kd   = this->get_parameter("yaw_kd").as_double();

        auto latest_qos = rclcpp::QoS(rclcpp::KeepLast(1))
                            .best_effort()
                            .durability_volatile();

        // subscription
        imu_subscription = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", latest_qos, std::bind(&YawController::imu_cb, this, _1));
        
        
    }
}
