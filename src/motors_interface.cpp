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

    MotorsInterface() : Node("motors_interface")
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
    int min_subs_period;
    int command_period;
    
    rclcpp::Subscription<common_interfaces::msg::VehicleMode>::SharedPtr subs_mode;
    rclcpp::Subscription<common_interfaces::msg::VehicleControl>::SharedPtr subs_autonomous;
    rclcpp::Subscription<common_interfaces::msg::VehicleControl>::SharedPtr subs_manual;

    rclcpp::TimerBase::SharedPtr timer_subs_mode_timeout;
    rclcpp::TimerBase::SharedPtr timer_subs_autonomous_timeout;
    rclcpp::TimerBase::SharedPtr timer_subs_manual_timeout;
    rclcpp::TimerBase::SharedPtr timer_subs_embrake_timeout;
    rclcpp::TimerBase::SharedPtr timer_command;

    rclcpp::Publisher<common_interfaces::msg::VehicleControl>::SharedPtr pub_info;
    rclcpp::Publisher<common_interfaces::msg::VehicleControl>::SharedPtr pub_sim;

    std::atomic<bool> subs_mode_ok;
    std::atomic<bool> subs_autonomous_ok;
    std::atomic<bool> subs_manual_ok;

    std::atomic<int> mode;
    std::atomic<float> manual_throttle;
    std::atomic<float> manual_steering_angle;
    std::atomic<float> autonomous_throttle;
    std::atomic<float> autonomous_steering_angle;    

    void timer_subs_mode_timeout_callback()
    {
      //RCLCPP_WARN(this->get_logger(), "Timeout timer_subs_mode");
      subs_mode_ok = false;
    }

    void timer_subs_autonomous_timeout_callback()
    {
      //RCLCPP_WARN(this->get_logger(), "Timeout timer_subs_autonomous");
      subs_autonomous_ok = false;
    }

    void timer_subs_manual_timeout_callback()
    {
      //RCLCPP_WARN(this->get_logger(), "Timeout timer_subs_manual");
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
      
      double throttle = 0.0;
      double steering_angle = 0.0;    

      if (subs_mode_ok) {
        switch (mode) {

          case 0: // neutral
            break;

          case 1: // manual
            if (subs_manual_ok) {
              throttle = manual_throttle;
              steering_angle = manual_steering_angle;
            } else {
              RCLCPP_WARN(this->get_logger(), "Mode=1 but no msgs on manaul topic");
            }
            break;

          case 2: // autonomous
            if (subs_autonomous_ok) {
              throttle = autonomous_throttle;
              steering_angle = autonomous_steering_angle;              
            } else {
              RCLCPP_WARN(this->get_logger(), "Mode=2 but no msgs on autonomous topic");
            }
            break;        
        }       
      } else {
        RCLCPP_WARN(this->get_logger(), "No msgs on mode topic");
      }

      RCLCPP_INFO(this->get_logger(), "Command->\tThrottle:%f\tSteeringAngle:%f", throttle, steering_angle);
      motors.SetThrottleAndSteer(throttle,steering_angle);

      // msg to simulator
      if (sim_publisher) {
        common_interfaces::msg::VehicleControl vehicle_control_msg;
        vehicle_control_msg.throttle = throttle;
        vehicle_control_msg.steering_angle = steering_angle;
        pub_sim->publish(vehicle_control_msg);
      }


    }

    void create_timers() {

      start_timer_subs_mode_timeout();
      start_timer_subs_autonomous_timeout();
      start_timer_subs_manual_timeout();

      start_timer_command();
    }

    void start_timer_subs_mode_timeout() {
      std::chrono::duration<int, std::milli> timeout_time(min_subs_period);
      timer_subs_mode_timeout = this->create_wall_timer(
        timeout_time, std::bind(&MotorsInterface::timer_subs_mode_timeout_callback, this)
        );
    }

    void start_timer_subs_autonomous_timeout() {
      std::chrono::duration<int, std::milli> timeout_time(min_subs_period);
      timer_subs_autonomous_timeout = this->create_wall_timer(
        timeout_time, std::bind(&MotorsInterface::timer_subs_autonomous_timeout_callback, this)
        );
    }

    void start_timer_subs_manual_timeout() {
      std::chrono::duration<int, std::milli> timeout_time(min_subs_period);
      timer_subs_manual_timeout = this->create_wall_timer(
        timeout_time, std::bind(&MotorsInterface::timer_subs_manual_timeout_callback, this)
        );
    }

    void start_timer_command() {
      std::chrono::duration<int, std::milli> timeout_time(command_period);
      timer_command = this->create_wall_timer(
        timeout_time, std::bind(&MotorsInterface::timer_command_callback, this)
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

    void create_publishers() {
      pub_info = this->create_publisher<common_interfaces::msg::VehicleControl>(cmd_info_topic_name, 10);
      pub_sim = this->create_publisher<common_interfaces::msg::VehicleControl>(cmd_sim_topic_name, 10);

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
      this->get_parameter("command_period", command_period);
      this->get_parameter("sim_publisher", sim_publisher);
      this->get_parameter("max_steering", max_steering);
      this->get_parameter("steer_pwm.center", steer_center);
      this->get_parameter("steer_pwm.dead_range", steer_dead_range);
      this->get_parameter("steer_pwm.range", steer_range);
      this->get_parameter("max_speed", max_speed);
      this->get_parameter("motor_pwm.center", motor_center);
      this->get_parameter("motor_pwm.dead_range", motor_dead_range);
      this->get_parameter("motor_pwm.range", motor_range);

      bool pca_ok = motors.SetupPCA9685(
        0x40,
        300,
        50,
        0,
        1,
        2
      );

      if (!pca_ok) {
        RCLCPP_ERROR(this->get_logger(), "PCA9685 NOT SETUP");
      }

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

      this->declare_parameter<int>("min_msg_period", 1000);
      this->declare_parameter<int>("command_period", 100);
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
