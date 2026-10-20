#include <memory>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class Imu_subscriber : public rclcpp::Node
{
  public:
    Imu_subscriber()
    : Node("sub_imu_node")
    {
      subscription_1 = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu_data", 20, std::bind(&Imu_subscriber::topic_callback, this, _1));
      subscription_2 = this->create_subscription<sensor_msgs::msg::Imu>(
      "/imu_offline_data", 20, std::bind(&Imu_subscriber::topic_callback, this, _1));
    }

  private:
    void topic_callback(const sensor_msgs::msg::Imu::SharedPtr imu_msg) const
    {
      RCLCPP_INFO(this->get_logger(), "加速度(x,y,z): '%lf' '%lf' '%lf'", imu_msg->linear_acceleration.x,
      imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
      RCLCPP_INFO(this->get_logger(), "欧拉角(r,p,y): '%lf' '%lf' '%lf'", imu_msg->angular_velocity.x,
      imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
    }
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_1;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_2;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Imu_subscriber>());
  rclcpp::shutdown();
  return 0;
}
