#ifndef COMPOSITION__MOTOR_COMPONENT_HPP_
#define COMPOSITION__MOTOR_COMPONENT_HPP_

#include "motors_interface/visibility_control.h"

#include "i2c_interfaces/srv/i2c_command.hpp"
#include "maila_msgs/msg/vehicle_control.hpp"
#include "maila_msgs/msg/vehicle_mode.hpp"

#include "rclcpp/rclcpp.hpp"

#include "motors_interface/pwm_motor.h"

#define PCA9685_ADDRESS 0x40
#define PCA9685_STEERING_SX_CH 2
#define PCA9685_STEERING_DX_CH 1
#define PCA9685_THROTTLE_CH 0
#define PCA9685_PIN_BASE 300
#define PCA9685_MAX_PWM 4096
#define PCA9685_HERTZ 50
#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE
#define LED0_ON_L 0x6
#define LEDALL_ON_L 0xFA

#include <functional>
using namespace std::placeholders;

namespace motor
{

class MotorComponent : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit MotorComponent(const rclcpp::NodeOptions & options);

private:
  
  rclcpp::Client<i2c_interfaces::srv::I2cCommand>::SharedPtr i2c_command;
  bool call_i2c_command_service(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Request> request, std::function<void(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response>)> fun);
  bool wait_for_i2c_command_service();

  rclcpp::Subscription<maila_msgs::msg::VehicleMode>::SharedPtr subs_mode;
  void subs_mode_callback(const maila_msgs::msg::VehicleMode::SharedPtr msg);
  rclcpp::TimerBase::SharedPtr timer_subs_mode_timeout;
  void timer_subs_mode_timeout_callback();
  void start_timer_subs_mode_timeout();
  
  rclcpp::Subscription<maila_msgs::msg::VehicleControl>::SharedPtr subs_autonomous;
  void subs_autonomous_callback(const maila_msgs::msg::VehicleControl::SharedPtr msg);
  rclcpp::TimerBase::SharedPtr timer_subs_autonomous_timeout;
  void timer_subs_autonomous_timeout_callback();
  void start_timer_subs_autonomous_timeout();

  rclcpp::Subscription<maila_msgs::msg::VehicleControl>::SharedPtr subs_manual;
  void subs_manual_callback(const maila_msgs::msg::VehicleControl::SharedPtr msg);
  rclcpp::TimerBase::SharedPtr timer_subs_manual_timeout;
  void timer_subs_manual_timeout_callback();
  void start_timer_subs_manual_timeout();

  rclcpp::Publisher<maila_msgs::msg::VehicleControl>::SharedPtr pub_info;
  
  rclcpp::TimerBase::SharedPtr timer_command;
  void timer_command_callback();
  void start_timer_command();


  void create_subscriptions();
  void create_publishers();
  void create_clients();

  void load_parameters();
  void initialize();

  void init_pca9685_step1();
  void init_pca9685_step2(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response> response);
  void init_pca9685_step3(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response> response);
  void init_pca9685_step4(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response> response);
  void init_pca9685_step5(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response> response);
  void init_pca9685_step6(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response> response);
  void init_pca9685_step7(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response> response);
  void init_pca9685_step8(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response> response);
  void init_pca9685_step9(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response> response);
  void pca9685_cmd_cb(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response> response);
  
  void send_pwm(uint8_t reg, int value);  
  
  
  // Topic name list
  std::string mode_topic_name;        // = "/autonomous_driver";
  std::string manual_cmd_topic_name;  // = "/hw/manual_cmd";
  std::string auto_cmd_topic_name;    // = "/hw/autonomous_cmd";
  std::string emergency_topic_name;   // = "/hw/emergency_brake";
  std::string cmd_info_topic_name;    // = "/hw/info";

  int msg_max_period;
  int command_period;
  int info_msg_rate;
  bool use_real_pca9685;

  float max_steering;
  float steering_pwm_range, steering_pwm_dead_range, steering_pwm_center;

  float max_throttle;
  float throttle_pwm_range, throttle_pwm_dead_range, throttle_pwm_center;

  PWMMotor steering_pwm;
  PWMMotor throttle_pwm;

  int pca9685_step;
  bool pca9685_initialized;
  int pca9685_settings;
  int pca9685_autoInc;
  int pca9685_sleep;
	int pca9685_wake;
	int pca9685_restart;
  int pca9685_prescale;

  std::atomic<bool> subs_mode_ok;
  std::atomic<bool> subs_autonomous_ok;
  std::atomic<bool> subs_manual_ok;

  std::atomic<int> mode;
  std::atomic<float> manual_throttle;
  std::atomic<float> manual_steering_angle;
  std::atomic<float> autonomous_throttle;
  std::atomic<float> autonomous_steering_angle;

  std::atomic<int> count_command_send; 

};

}  // namespace motor

#endif  // COMPOSITION__MOTOR_COMPONENT_HPP_
