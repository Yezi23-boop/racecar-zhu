//
// racecar_driver_node_one.cpp
//

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

#include "../include/racecar_driver.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <string>

class RacecarDriver : public rclcpp::Node
{
public:
  RacecarDriver()
  : Node("racecar_driver")
  {
    declare_parameter<std::string>("serial_port", "/dev/car");
    declare_parameter<int>("baud_rate", 38400);

    // 起步补偿参数，均可在 launch 文件中覆盖。
    declare_parameter<bool>("enable_startup_boost", true);
    declare_parameter<int>("neutral_motor_pwm", 1500);
    declare_parameter<int>("neutral_servo_pwm", 800);
    declare_parameter<int>("startup_boost_pwm", 1565);
    declare_parameter<int>("startup_trigger_max_pwm", 1556);
    declare_parameter<double>("startup_boost_duration", 0.30);
    declare_parameter<double>("startup_rearm_duration", 0.35);
    declare_parameter<double>("command_timeout", 0.50);
    declare_parameter<double>("output_rate", 50.0);

    // 导航 Twist 到下位机 PWM 的换算参数。
    declare_parameter<double>("navigation_speed_scale", 100.0);
    declare_parameter<double>("navigation_steering_scale", 400.0);
    declare_parameter<int>("motor_pwm_min", 1000);
    declare_parameter<int>("motor_pwm_max", 2000);
    declare_parameter<int>("servo_pwm_min", 600);
    declare_parameter<int>("servo_pwm_max", 1000);
    declare_parameter<int>("neutral_deadband_pwm", 1);

    // 转向过冲：每一次舵机目标变化，先朝变化方向多打一段，再落到目标，抵消机构松动。
    declare_parameter<bool>("steer_overshoot_enable", true);
    declare_parameter<int>("steer_overshoot_pwm", 100);
    declare_parameter<double>("steer_overshoot_sec", 0.40);

    auto serial_port = get_parameter("serial_port").as_string();
    const auto baud_rate = get_parameter("baud_rate").as_int();
    enable_startup_boost_ = get_parameter("enable_startup_boost").as_bool();
    neutral_motor_pwm_ = get_parameter("neutral_motor_pwm").as_int();
    neutral_servo_pwm_ = get_parameter("neutral_servo_pwm").as_int();
    startup_boost_pwm_ = get_parameter("startup_boost_pwm").as_int();
    startup_trigger_max_pwm_ = get_parameter("startup_trigger_max_pwm").as_int();
    startup_boost_duration_ = get_parameter("startup_boost_duration").as_double();
    startup_rearm_duration_ = get_parameter("startup_rearm_duration").as_double();
    command_timeout_ = get_parameter("command_timeout").as_double();
    output_rate_ = get_parameter("output_rate").as_double();
    navigation_speed_scale_ = get_parameter("navigation_speed_scale").as_double();
    navigation_steering_scale_ = get_parameter("navigation_steering_scale").as_double();
    motor_pwm_min_ = get_parameter("motor_pwm_min").as_int();
    motor_pwm_max_ = get_parameter("motor_pwm_max").as_int();
    servo_pwm_min_ = get_parameter("servo_pwm_min").as_int();
    servo_pwm_max_ = get_parameter("servo_pwm_max").as_int();
    neutral_deadband_pwm_ = get_parameter("neutral_deadband_pwm").as_int();
    steer_overshoot_enable_ = get_parameter("steer_overshoot_enable").as_bool();
    steer_overshoot_pwm_ = get_parameter("steer_overshoot_pwm").as_int();
    steer_overshoot_sec_ = get_parameter("steer_overshoot_sec").as_double();

    startup_boost_duration_ = std::max(0.0, startup_boost_duration_);
    startup_rearm_duration_ = std::max(0.0, startup_rearm_duration_);
    command_timeout_ = std::max(0.1, command_timeout_);
    output_rate_ = std::max(1.0, output_rate_);
    neutral_deadband_pwm_ = std::max(0, neutral_deadband_pwm_);
    steer_overshoot_pwm_ = std::max(0, steer_overshoot_pwm_);
    steer_overshoot_sec_ = std::max(0.0, steer_overshoot_sec_);
    startup_boost_pwm_ = std::max(startup_boost_pwm_, neutral_motor_pwm_);
    startup_trigger_max_pwm_ = std::max(startup_trigger_max_pwm_, neutral_motor_pwm_);

    art_racecar_init(static_cast<int>(baud_rate), serial_port.data());

    desired_motor_pwm_ = neutral_motor_pwm_;
    desired_servo_pwm_ = neutral_servo_pwm_;
    last_target_servo_pwm_ = neutral_servo_pwm_;
    steer_overshoot_until_ = now();
    last_command_time_ = now();
    neutral_since_ = now();
    boost_until_ = now();

    navigation_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 1,
      std::bind(&RacecarDriver::navigationCallback, this, std::placeholders::_1));

    teleop_subscription_ = create_subscription<geometry_msgs::msg::Twist>(
      "/teleop_cmd_vel", 1,
      std::bind(&RacecarDriver::teleopCallback, this, std::placeholders::_1));

    const auto timer_period = std::chrono::duration<double>(1.0 / output_rate_);
    output_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(timer_period),
      std::bind(&RacecarDriver::outputTimerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "底盘驱动已启动：起步补偿=%s，起步PWM=%d，持续%.3f秒，触发上限=%d",
      enable_startup_boost_ ? "开启" : "关闭", startup_boost_pwm_,
      startup_boost_duration_, startup_trigger_max_pwm_);
  }

  ~RacecarDriver() override
  {
    // 节点退出时尽力发送一次停车和舵机回中指令。
    send_cmd(
      static_cast<uint16_t>(neutral_motor_pwm_),
      static_cast<uint16_t>(neutral_servo_pwm_));
  }

private:
  void navigationCallback(const geometry_msgs::msg::Twist::SharedPtr twist)
  {
    const int motor_pwm = static_cast<int>(std::lround(
        neutral_motor_pwm_ + twist->linear.x * navigation_speed_scale_));
    const int servo_pwm = static_cast<int>(std::lround(
        neutral_servo_pwm_ + twist->angular.z * navigation_steering_scale_));
    updateDesiredCommand(motor_pwm, servo_pwm, "导航");
  }

  void teleopCallback(const geometry_msgs::msg::Twist::SharedPtr twist)
  {
    // 遥控沿用原协议：linear.x 为电机 PWM，angular.z 为 0~180 度的舵机角度。
    // 角度按实测行程映射：90 度 = 中位 800µs，20/160 度 = 左右满舵（1000/600µs）。
    const int motor_pwm = static_cast<int>(std::lround(twist->linear.x));
    const int servo_pwm = static_cast<int>(std::lround(
        neutral_servo_pwm_ + (90.0 - twist->angular.z) *
        (servo_pwm_max_ - servo_pwm_min_) / 2.0 / 70.0));
    updateDesiredCommand(motor_pwm, servo_pwm, "遥控");
  }

  void updateDesiredCommand(int motor_pwm, int servo_pwm, const char * source)
  {
    const auto current_time = now();
    desired_motor_pwm_ = std::clamp(motor_pwm, motor_pwm_min_, motor_pwm_max_);
    desired_servo_pwm_ = std::clamp(servo_pwm, servo_pwm_min_, servo_pwm_max_);
    last_command_time_ = current_time;
    timeout_stop_reported_ = false;

    const bool is_neutral =
      std::abs(desired_motor_pwm_ - neutral_motor_pwm_) <= neutral_deadband_pwm_;
    const bool is_forward = desired_motor_pwm_ > neutral_motor_pwm_ + neutral_deadband_pwm_;

    if (is_neutral) {
      // 停车命令优先级最高，立即取消未结束的起步补偿。
      boost_active_ = false;
      if (!neutral_tracking_) {
        neutral_tracking_ = true;
        neutral_since_ = current_time;
      }
      sendCurrentCommand();
      return;
    }

    neutral_tracking_ = false;

    if (!is_forward) {
      // 当前不对倒车进行起步补偿。
      boost_active_ = false;
      boost_armed_ = false;
      return;
    }

    if (
      enable_startup_boost_ && boost_armed_ &&
      desired_motor_pwm_ <= startup_trigger_max_pwm_)
    {
      boost_active_ = true;
      boost_armed_ = false;
      boost_until_ = current_time + rclcpp::Duration::from_seconds(startup_boost_duration_);
      RCLCPP_INFO(
        get_logger(), "%s起步补偿：目标PWM=%d，暂时输出PWM=%d，持续%.3f秒",
        source, desired_motor_pwm_, std::max(desired_motor_pwm_, startup_boost_pwm_),
        startup_boost_duration_);
    }
  }

  void outputTimerCallback()
  {
    const auto current_time = now();

    if ((current_time - last_command_time_).seconds() > command_timeout_) {
      desired_motor_pwm_ = neutral_motor_pwm_;
      desired_servo_pwm_ = neutral_servo_pwm_;
      boost_active_ = false;
      if (!neutral_tracking_) {
        neutral_tracking_ = true;
        neutral_since_ = current_time;
      }
      if (!timeout_stop_reported_) {
        RCLCPP_WARN(get_logger(), "速度指令超时，底盘自动停车");
        timeout_stop_reported_ = true;
      }
    }

    if (boost_active_ && current_time >= boost_until_) {
      boost_active_ = false;
      RCLCPP_INFO(get_logger(), "起步补偿结束，恢复目标PWM=%d", desired_motor_pwm_);
    }

    if (
      neutral_tracking_ && !boost_armed_ &&
      (current_time - neutral_since_).seconds() >= startup_rearm_duration_)
    {
      boost_armed_ = true;
      RCLCPP_DEBUG(get_logger(), "起步补偿已重新就绪");
    }

    sendCurrentCommand();
  }

  void sendCurrentCommand()
  {
    int output_motor_pwm = desired_motor_pwm_;
    if (boost_active_) {
      output_motor_pwm = std::max(output_motor_pwm, startup_boost_pwm_);
    }

    output_motor_pwm = std::clamp(output_motor_pwm, motor_pwm_min_, motor_pwm_max_);

    int output_servo_pwm = desired_servo_pwm_;
    bool overshoot_output = false;
    if (steer_overshoot_enable_ && steer_overshoot_pwm_ > 0 && steer_overshoot_sec_ > 0.0) {
      const auto current_time = now();
      if (desired_servo_pwm_ != last_target_servo_pwm_) {
        // 目标变了：先朝变化方向多打一段（允许越过舵机极限），保持后再落到新目标。
        const int direction = desired_servo_pwm_ > last_target_servo_pwm_ ? 1 : -1;
        const int overshoot_pwm = desired_servo_pwm_ + direction * steer_overshoot_pwm_;
        last_target_servo_pwm_ = desired_servo_pwm_;
        steer_overshoot_output_pwm_ = overshoot_pwm;
        steer_overshoot_active_ = true;
        steer_overshoot_until_ =
          current_time + rclcpp::Duration::from_seconds(steer_overshoot_sec_);
        RCLCPP_INFO(
          get_logger(), "转向过冲：%d -> %d µs，保持 %.2f 秒后到目标 %d",
          last_output_servo_pwm_, steer_overshoot_output_pwm_,
          steer_overshoot_sec_, desired_servo_pwm_);
      }
      if (steer_overshoot_active_) {
        if (current_time < steer_overshoot_until_) {
          output_servo_pwm = steer_overshoot_output_pwm_;
          overshoot_output = true;
        } else {
          steer_overshoot_active_ = false;
          RCLCPP_INFO(get_logger(), "转向过冲结束，到达目标 %d", desired_servo_pwm_);
        }
      }
    }
    if (!overshoot_output) {
      output_servo_pwm = std::clamp(output_servo_pwm, servo_pwm_min_, servo_pwm_max_);
    }

    send_cmd(
      static_cast<uint16_t>(output_motor_pwm),
      static_cast<uint16_t>(output_servo_pwm));
    last_output_servo_pwm_ = output_servo_pwm;

    const char * note = steer_overshoot_active_
      ? "（转向过冲）" : (boost_active_ ? "（起步补偿中）" : "");
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "底盘输出：电机PWM=%d，舵机PWM=%d，目标电机PWM=%d%s",
      output_motor_pwm, output_servo_pwm, desired_motor_pwm_, note);
  }

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr navigation_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_subscription_;
  rclcpp::TimerBase::SharedPtr output_timer_;

  bool enable_startup_boost_{true};
  bool boost_active_{false};
  bool boost_armed_{true};
  bool neutral_tracking_{true};
  bool timeout_stop_reported_{false};

  int neutral_motor_pwm_{1500};
  int neutral_servo_pwm_{800};
  int startup_boost_pwm_{1555};
  int startup_trigger_max_pwm_{1546};
  int desired_motor_pwm_{1500};
  int desired_servo_pwm_{1500};
  int motor_pwm_min_{1000};
  int motor_pwm_max_{2000};
  int servo_pwm_min_{600};
  int servo_pwm_max_{1000};
  int neutral_deadband_pwm_{1};

  bool steer_overshoot_enable_{true};
  int steer_overshoot_pwm_{100};
  double steer_overshoot_sec_{0.40};
  bool steer_overshoot_active_{false};
  int steer_overshoot_output_pwm_{0};
  int last_target_servo_pwm_{0};
  int last_output_servo_pwm_{0};

  double startup_boost_duration_{0.25};
  double startup_rearm_duration_{0.30};
  double command_timeout_{0.50};
  double output_rate_{50.0};
  double navigation_speed_scale_{100.0};
  double navigation_steering_scale_{400.0};

  rclcpp::Time last_command_time_;
  rclcpp::Time neutral_since_;
  rclcpp::Time boost_until_;
  rclcpp::Time steer_overshoot_until_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RacecarDriver>());
  rclcpp::shutdown();
  return 0;
}
