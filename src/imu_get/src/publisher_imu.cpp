#include <chrono>
#include <math.h>
#include "serial/serial.h"
#include <memory.h>
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "nav_msgs/msg/odometry.hpp"
#include <string>
#include "rclcpp/rclcpp.hpp"

#include "transform.hpp"

serial::Serial ser;

using namespace std::chrono_literals;

class publisher_imu_node : public rclcpp::Node
{
public:
    std::string port;
    int baudrate;
    transform_imu imu_fetch;
    rclcpp::Time last_time_;
    int msg_count_;
    bool calibration_done_;
    
public:
    publisher_imu_node()
        : Node("publisher_imu_node"),
          last_time_(this->get_clock()->now()),
          msg_count_(0),
          calibration_done_(false)
    {
        // 100Hz = 10ms周期
        int output_hz = 100;
        auto timer_period = std::chrono::milliseconds(10);
        
        port = "/dev/imu";
        baudrate = 115200;
        
        try
        {
            ser.setPort(port);
            ser.setBaudrate(baudrate);
            serial::Timeout to = serial::Timeout::simpleTimeout(100);
            ser.setTimeout(to);
            ser.open();
            
            // 清空缓冲区
            ser.flushInput();
            ser.flushOutput();
            
            RCLCPP_INFO(this->get_logger(), "IMU port %s opened at %d baud", 
                       port.c_str(), baudrate);
        }
        catch (serial::IOException &e)
        {
            RCLCPP_FATAL(this->get_logger(), "Unable to open port %s: %s", 
                        port.c_str(), e.what());
            return;
        }
        catch (serial::SerialException &e)
        {
            RCLCPP_FATAL(this->get_logger(), "Serial exception: %s", e.what());
            return;
        }

        if (!ser.isOpen())
        {
            RCLCPP_FATAL(this->get_logger(), "Serial port not open");
            return;
        }

        // 创建发布者，队列大小100确保不丢数据
        pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("/imu_data", 100);
        pub_imu_offline = this->create_publisher<sensor_msgs::msg::Imu>("/IMU_data", 100);
        
        // 创建校准服务（可选）
        // calibrate_srv_ = this->create_service<std_srvs::srv::Empty>(
        //     "calibrate_imu", std::bind(&publisher_imu_node::calibrate_callback, this, _1, _2));

        // 创建定时器
        timer_ = this->create_wall_timer(timer_period, 
            std::bind(&publisher_imu_node::timer_callback, this));
        
        RCLCPP_INFO(this->get_logger(), "IMU node started at %d Hz", output_hz);
        RCLCPP_INFO(this->get_logger(), "IMU calibration applied");
        
        // 打印校准参数
        double acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z;
        imu_fetch.get_calibration(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
        RCLCPP_INFO(this->get_logger(), 
                   "Calibration offsets - Acc: (%.3f, %.3f, %.3f), Gyro: (%.5f, %.5f, %.5f)",
                   acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
    }

    ~publisher_imu_node()
    {
        if (ser.isOpen())
        {
            ser.close();
            RCLCPP_INFO(this->get_logger(), "Serial port closed");
        }
    }

private:
    void timer_callback()
    {
        int available = ser.available();
        
        if (available > 0)
        {
            std::vector<unsigned char> read_buf(available);
            int bytes_read = ser.read(&read_buf[0], available);
            
            if (bytes_read > 0)
            {
                rclcpp::Time now = this->get_clock()->now();
                
                // 转换为char类型
                std::vector<char> read_buf_char(bytes_read);
                for(int i = 0; i < bytes_read; i++)
                {
                    read_buf_char[i] = static_cast<char>(read_buf[i]);
                }
                
                // 解析数据（已包含校准）
                imu_fetch.FetchData(read_buf_char, bytes_read);
                
                // 发布数据
                publish_imu_data(now);
                
                // 调试输出（每100次输出一次）
                if (++msg_count_ % 100 == 0)
                {
                    double rate = 100.0 / ((now - last_time_).seconds());
                    RCLCPP_DEBUG(this->get_logger(), 
                        "IMU Rate: %.1f Hz", rate);
                    
                    // 输出校准后的数据示例
                    RCLCPP_DEBUG(this->get_logger(),
                        "Calibrated Data - Acc: (%.3f, %.3f, %.3f) m/s², "
                        "Gyro: (%.5f, %.5f, %.5f) rad/s",
                        imu_fetch.acc.x, imu_fetch.acc.y, imu_fetch.acc.z,
                        imu_fetch.gyro.x, imu_fetch.gyro.y, imu_fetch.gyro.z);
                    
                    last_time_ = now;
                }
            }
        }
    }
    
    void publish_imu_data(const rclcpp::Time& now)
    {
        sensor_msgs::msg::Imu imu_data;
        sensor_msgs::msg::Imu imu_offline_data;
        
        // ============== imu_data (/imu_data) ==============
        imu_data.header.stamp = now;
        imu_data.header.frame_id = "imu_link";
        
        // 加速度（已在校准中处理）
        imu_data.linear_acceleration.x = imu_fetch.acc.x;
        imu_data.linear_acceleration.y = imu_fetch.acc.y;
        imu_data.linear_acceleration.z = imu_fetch.acc.z;
        
        // 角速度（已在校准中处理）
        imu_data.angular_velocity.x = imu_fetch.gyro.x;
        imu_data.angular_velocity.y = imu_fetch.gyro.y;
        imu_data.angular_velocity.z = imu_fetch.gyro.z;
        
        // 四元数
        tf2::Quaternion quaternion;
        quaternion.setRPY(imu_fetch.angle.r, imu_fetch.angle.p, imu_fetch.angle.y);
        quaternion.normalize();
        
        imu_data.orientation.x = quaternion.x();
        imu_data.orientation.y = quaternion.y();
        imu_data.orientation.z = quaternion.z();
        imu_data.orientation.w = quaternion.w();
        
        // 设置协方差矩阵
        set_covariance_matrix(imu_data);
        
        // ============== imu_offline_data (/IMU_data) ==============
        imu_offline_data.header.stamp = now;
        imu_offline_data.header.frame_id = "IMU_link";  // Cartographer使用
        
        // 复制校准后的数据
        imu_offline_data.linear_acceleration = imu_data.linear_acceleration;
        imu_offline_data.angular_velocity = imu_data.angular_velocity;
        imu_offline_data.orientation = imu_data.orientation;
        
        // 复制协方差矩阵
        for (int i = 0; i < 9; i++) {
            imu_offline_data.linear_acceleration_covariance[i] = 
                imu_data.linear_acceleration_covariance[i];
            imu_offline_data.angular_velocity_covariance[i] = 
                imu_data.angular_velocity_covariance[i];
            imu_offline_data.orientation_covariance[i] = 
                imu_data.orientation_covariance[i];
        }
        
        // 发布消息
        pub_imu->publish(imu_data);
        pub_imu_offline->publish(imu_offline_data);
    }
    
    void set_covariance_matrix(sensor_msgs::msg::Imu& imu_msg)
    {
        // 对角线协方差矩阵
        // 加速度协方差 (m/s²)²
        imu_msg.linear_acceleration_covariance[0] = 0.04;  // x轴
        imu_msg.linear_acceleration_covariance[4] = 0.04;  // y轴
        imu_msg.linear_acceleration_covariance[8] = 0.04;  // z轴
        
        // 角速度协方差 (rad/s)²
        imu_msg.angular_velocity_covariance[0] = 0.02;  // x轴
        imu_msg.angular_velocity_covariance[4] = 0.02;  // y轴
        imu_msg.angular_velocity_covariance[8] = 0.02;  // z轴
        
        // 方向协方差
        imu_msg.orientation_covariance[0] = 0.05;  // Roll
        imu_msg.orientation_covariance[4] = 0.05;  // Pitch
        imu_msg.orientation_covariance[8] = 0.05;  // Yaw
        
        // 非对角线元素设为0
        for (int i = 0; i < 9; i++) {
            if (i != 0 && i != 4 && i != 8) {
                imu_msg.linear_acceleration_covariance[i] = 0;
                imu_msg.angular_velocity_covariance[i] = 0;
                imu_msg.orientation_covariance[i] = 0;
            }
        }
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_offline;
    // rclcpp::Service<std_srvs::srv::Empty>::SharedPtr calibrate_srv_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    
    printf("=========================================\n");
    printf("Starting IMU Publisher Node with Calibration\n");
    printf("=========================================\n");
    printf("IMU Port: /dev/imu\n");
    printf("Baudrate: 115200\n");
    printf("Frequency: 100Hz\n");
    printf("Calibration Applied\n");
    printf("=========================================\n");
    
    auto node = std::make_shared<publisher_imu_node>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    
    try {
        executor.spin();
    } catch (const std::exception& e) {
        printf("IMU node exception: %s\n", e.what());
    }
    
    printf("\nIMU node shutdown\n");
    rclcpp::shutdown();
    return 0;
}
