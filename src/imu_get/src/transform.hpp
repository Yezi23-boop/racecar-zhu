#include <string>
#include <cctype>
#include <cfloat>
#include <cmath>
#include <vector>
#include <cstdint>
#include <cstdio>

class transform_imu
{
private:
    const char *stcTime = nullptr;
    const char *stcAcc = nullptr;
    const char *stcGyro = nullptr;
    const char *stcAngle = nullptr;

    // 最终校准参数（基于你的最新数据）
    struct Calibration {
        // 加速度偏移量（静止时：x=0, y=0, z=9.81）
        // 计算：offset = 原始值 - 目标值
        double acc_x_offset = 0.999;    // 目标：x=0, 当前：x≈0.082
        double acc_y_offset = 0.307;    // 目标：y=0, 当前：y≈-0.058
        double acc_z_offset = 0.014;    // 目标：z=9.81, 当前：z≈9.767
        
        // 陀螺仪偏移量（静止时：x=0, y=0, z=0）
        double gyro_x_offset = -0.00213; // 目标：x=0, 当前：x≈-0.00213
        double gyro_y_offset = 0.00106;  // 目标：y=0, 当前：y≈0.00106
        double gyro_z_offset = 0.0;      // 目标：z=0, 当前：z≈-4.7e-06
    } calib;

public:
    struct Acc
    {
        double x;
        double y;
        double z;
    } acc{0,0,0};
    struct Gyro
    {
        double x;
        double y;
        double z;
    } gyro{0,0,0};
    struct Angle
    {
        double r;
        double p;
        double y;
    } angle{0,0,0};

    void FetchData(std::vector<char> &data, int usLength)
    {
        int index = 0;
        auto read_s16 = [&](int off)->int16_t {
            if (off + 1 >= static_cast<int>(data.size())) return 0;
            uint8_t low = static_cast<uint8_t>(data[off]);
            uint8_t high = static_cast<uint8_t>(data[off+1]);
            return static_cast<int16_t>((high << 8) | low);
        };

        while (usLength >= 11)
        {
            if (static_cast<uint8_t>(data[index]) != 0x55)
            {
                index++;
                usLength--;
                continue;
            }

            uint8_t data_type = static_cast<uint8_t>(data[index + 1]);
            
            switch (data_type)
            {
                case 0x50: // 时间
                    stcTime = &data[index + 2];
                    break;
                    
                case 0x51: // 加速度
                {
                    // 读取原始数据
                    double raw_x = read_s16(index + 2) / 32768.0 * 16.0 * 9.8;
                    double raw_y = read_s16(index + 4) / 32768.0 * 16.0 * 9.8;
                    double raw_z = read_s16(index + 6) / 32768.0 * 16.0 * 9.8;
                    
                    // 应用最终校准
                    acc.x = raw_x - calib.acc_x_offset;
                    acc.y = raw_y - calib.acc_y_offset;
                    acc.z = raw_z - calib.acc_z_offset;
                    break;
                }
                    
                case 0x52: // 陀螺仪
                {
                    // 读取原始数据
                    double raw_x = read_s16(index + 2) / 32768.0 * 2000.0 / 180.0 * M_PI;
                    double raw_y = read_s16(index + 4) / 32768.0 * 2000.0 / 180.0 * M_PI;
                    double raw_z = read_s16(index + 6) / 32768.0 * 2000.0 / 180.0 * M_PI;
                    
                    // 应用最终校准
                    gyro.x = raw_x - calib.gyro_x_offset;
                    gyro.y = raw_y - calib.gyro_y_offset;
                    gyro.z = raw_z - calib.gyro_z_offset;
                    break;
                }
                    
                case 0x53: // 角度（Euler）
                    angle.r = read_s16(index + 2) / 32768.0 * M_PI;
                    angle.p = read_s16(index + 4) / 32768.0 * M_PI;
                    angle.y = read_s16(index + 6) / 32768.0 * M_PI;
                    break;
            }

            usLength -= 11;
            index += 11;
        }
    }
    
    // 校准方法
    void calibrate(double new_acc_x_offset, double new_acc_y_offset, double new_acc_z_offset,
                   double new_gyro_x_offset, double new_gyro_y_offset, double new_gyro_z_offset)
    {
        calib.acc_x_offset = new_acc_x_offset;
        calib.acc_y_offset = new_acc_y_offset;
        calib.acc_z_offset = new_acc_z_offset;
        calib.gyro_x_offset = new_gyro_x_offset;
        calib.gyro_y_offset = new_gyro_y_offset;
        calib.gyro_z_offset = new_gyro_z_offset;
    }
    
    // 获取当前校准值
    void get_calibration(double& acc_x, double& acc_y, double& acc_z,
                        double& gyro_x, double& gyro_y, double& gyro_z)
    {
        acc_x = calib.acc_x_offset;
        acc_y = calib.acc_y_offset;
        acc_z = calib.acc_z_offset;
        gyro_x = calib.gyro_x_offset;
        gyro_y = calib.gyro_y_offset;
        gyro_z = calib.gyro_z_offset;
    }
};
