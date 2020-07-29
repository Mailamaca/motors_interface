#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <atomic>


#include "motors_interface/pca9685_motors.h"

#include "common_interfaces/msg/vehicle_control.hpp"
#include "common_interfaces/msg/vehicle_mode.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"


using namespace std::chrono_literals;
using std::placeholders::_1;


class MotorsInterface : public rclcpp::Node
{
  public:

    MotorsInterface() : Node("motors_interface"), count_(0)
    {
      declare_parameters();
      load_parameters();    

      create_publishers();
      create_subscriptions();
      create_timers();      
      
    }
  
  private:

    PCA9685Motors motors;

    // Topic name list
    std::string manual_cmd_topic_name;  // = "/hw/manual_cmd";
    std::string auto_cmd_topic_name;    //   = "/hw/autonomous_cmd";
    std::string cmd_info_topic_name;    //   = "/hw/info";
    std::string emergency_topic_name;   //  = "/hw/emergency_brake";
    std::string autonomous_mode_topic_name; // = "/autonomous_driver";
    std::string cmd_sim_topic_name;     //      = "/sim/cmd";

    bool sim_publisher = false;
    double min_subs_period;
    Duration timeout_duration;
    
    rclcpp::Subscription<common_interfaces::msg::VehicleMode>::SharedPtr subs_mode;
    rclcpp::Subscription<common_interfaces::msg::VehicleControl>::SharedPtr subs_autonomous;
    rclcpp::Subscription<common_interfaces::msg::VehicleControl>::SharedPtr subs_manual;

    rclcpp::TimerBase::SharedPtr timer_subs_mode_timeout;
    rclcpp::TimerBase::SharedPtr timer_subs_autonomous_timeout;
    rclcpp::TimerBase::SharedPtr timer_subs_manual_timeout;
    rclcpp::TimerBase::SharedPtr timer_subs_embrake_timeout;
    rclcpp::TimerBase::SharedPtr timer_command;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_info;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_sim;

    std::atomic<bool> subs_mode_ok;
    std::atomic<bool> subs_autonomous_ok;
    std::atomic<bool> subs_manual_ok;

    std::atomic<int> mode;
    std::atomic<float> manual_throttle;
    std::atomic<float> manual_steering_angle;
    std::atomic<float> autonomous_throttle;
    std::atomic<float> autonomous_steering_angle;
    
    size_t count_;

    void timer_subs_mode_timeout_callback()
    {
      RCLCPP_INFO(this->get_logger(), "Timeout timer_subs_mode");
      subs_mode_ok = false;
    }

    void timer_subs_autonomous_timeout_callback()
    {
      RCLCPP_INFO(this->get_logger(), "Timeout timer_subs_autonomous");
      subs_autonomous_ok = false;
    }

    void timer_subs_manual_timeout_callback()
    {
      RCLCPP_INFO(this->get_logger(), "Timeout timer_subs_manual");
      subs_manual_ok = false;
    }

    void subs_mode_callback(const common_interfaces::msg::VehicleMode::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "subs_mode_callback");
      subs_mode_ok = true;
      mode = msg->mode;

      start_timer_subs_mode_timeout();
    }

    void subs_autonomous_callback(const common_interfaces::msg::VehicleControl::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "subs_autonomous_callback");
      subs_autonomous_ok = true;
      autonomous_throttle = msg->throttle;
      autonomous_steering_angle = msg->steering_angle;

      start_timer_subs_autonomous_timeout();
    }

    void subs_manual_callback(const common_interfaces::msg::VehicleControl::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "subs_manual_callback");
      subs_manual_ok = true;
      manual_throttle = msg->throttle;
      manual_steering_angle = msg->steering_angle;

      start_timer_subs_manual_timeout();
    }

    void timer_command_callback()
    {
      RCLCPP_INFO(this->get_logger(), "Timeout timer_command_callback");

      if (subs_mode_ok) {
        switch (mode) {

          case 0: // neutral
            motors.SetThrottleAndSteer(0.0,0.0);
            break;

          case 1: // manual
            if (subs_manual_ok) {
              motors.SetThrottleAndSteer(manual_throttle,manual_steering_angle);
            } else {
              motors.SetThrottleAndSteer(0.0,0.0);
            }
            break;

          case 2: // autonomous
            if (subs_autonomous_ok) {
              motors.SetThrottleAndSteer(manual_throttle,manual_steering_angle);
            } else {
              motors.SetThrottleAndSteer(0.0,0.0);
            }
            break;        
        }           

      } else {
        motors.SetThrottleAndSteer(0.0,0.0);
      }
    }

    void create_publishers() {
      pub_info = this->create_publisher<std_msgs::msg::String>(cmd_info_topic_name, 10);
      pub_sim = this->create_publisher<std_msgs::msg::String>(cmd_sim_topic_name, 10);

    }

    void create_timers() {

      start_timer_subs_mode_timeout();
      start_timer_subs_autonomous_timeout();
      start_timer_subs_manual_timeout();
      
      /// TODO: set parameter for timer command
      timer_command = this->create_wall_timer(
        50ms, std::bind(&MotorsInterface::timer_command_callback, this)
        );
      
    }

    void start_timer_subs_mode_timeout() {
      timer_subs_mode_timeout = this->create_wall_timer(
        min_subs_period, std::bind(&MotorsInterface::timer_subs_mode_timeout_callback, this)
        );
    }

    void start_timer_subs_autonomous_timeout() {
      timer_subs_autonomous_timeout = this->create_wall_timer(
        min_subs_period, std::bind(&MotorsInterface::timer_subs_autonomous_timeout_callback, this)
        );
    }

    void start_timer_subs_manual_timeout() {
      timer_subs_manual_timeout = this->create_wall_timer(
        min_subs_period, std::bind(&MotorsInterface::timer_subs_manual_timeout_callback, this)
        );
    }

    void create_subscriptions() {
      subs_mode = this->create_subscription<common_interfaces::msg::VehicleMode>(
        autonomous_mode_topic_name, 10, std::bind(&MotorsInterface::subs_mode_callback, this, _1));

      subs_autonomous = this->create_subscription<common_interfaces::msg::VehicleControl>(
        auto_cmd_topic_name, 10, std::bind(&MotorsInterface::subs_autonomous_callback, this, _1));

      subs_manual = this->create_subscription<common_interfaces::msg::VehicleControl>(
        manual_cmd_topic_name, 10, std::bind(&MotorsInterface::subs_manual_callback, this, _1));
    }

    void load_parameters() {

      float max_steering;
      float steer_range, steer_dead_range, steer_center;

      float max_speed;
      float motor_range, motor_dead_range, motor_center;

      this->get_parameter("topics.manual_cmd", manual_cmd_topic_name);
      this->get_parameter("topics.auto_cmd", auto_cmd_topic_name);
      this->get_parameter("topics.cmd_info", cmd_info_topic_name);
      this->get_parameter("topics.emergency", emergency_topic_name);
      this->get_parameter("topics.autonomous_mode", autonomous_mode_topic_name);
      this->get_parameter("topics.cmd_sim", cmd_sim_topic_name);
      this->get_parameter("min_msg_period", min_subs_period);
      this->get_parameter("sim_publisher", sim_publisher);
      this->get_parameter("max_steering", max_steering);
      this->get_parameter("steer_pwm.center", steer_center);
      this->get_parameter("steer_pwm.dead_range", steer_dead_range);
      this->get_parameter("steer_pwm.range", steer_range);
      this->get_parameter("max_speed", max_speed);
      this->get_parameter("motor_pwm.center", motor_center);
      this->get_parameter("motor_pwm.dead_range", motor_dead_range);
      this->get_parameter("motor_pwm.range", motor_range);

      motors.SetPCA9685Params(
        0x40,
        300,
        50,
        0,
        1,
        2
      );

      motors.SetSteerMotorParams(
        max_steering,
        steer_range,
        steer_dead_range,
        steer_center,
        4096
      );

      motors.SetSpeedMotorParams(
        max_speed,
        motor_range,
        motor_dead_range,
        motor_center,
        4096
      );

    }

    void declare_parameters() {
      this->declare_parameter<std::string>("topics.manual_cmd", "");
      this->declare_parameter<std::string>("topics.auto_cmd", "");
      this->declare_parameter<std::string>("topics.cmd_info", "");
      this->declare_parameter<std::string>("topics.emergency", "");
      this->declare_parameter<std::string>("topics.autonomous_mode", "");
      this->declare_parameter<std::string>("topics.cmd_sim", "");

      this->declare_parameter<double>("min_msg_period", 1);
      this->declare_parameter<bool>("sim_publisher", false);

      this->declare_parameter<float>("max_steering", 1);
      this->declare_parameter<float>("steer_pwm.center", 0);
      this->declare_parameter<float>("steer_pwm.dead_range", 0);
      this->declare_parameter<float>("steer_pwm.range", 1);

      this->declare_parameter<float>("max_speed", 1);
      this->declare_parameter<float>("motor_pwm.center", 0);
      this->declare_parameter<float>("motor_pwm.dead_range", 0);
      this->declare_parameter<float>("motor_pwm.range", 1);
    }

  
    
    

  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorsInterface>());
    rclcpp::shutdown();
    return 0;
  }
