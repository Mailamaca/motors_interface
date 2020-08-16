#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <atomic>


#include "motors_interface/pca9685_motors.h"

#include "maila_msgs/msg/vehicle_control.hpp"
#include "maila_msgs/msg/vehicle_mode.hpp"

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
      load_parameters_and_initialization();    

      create_publishers();
      create_subscriptions();
      create_timers();      
      
    }
  
  private:

    PCA9685Motors motors;

    // Topic name list
    std::string mode_topic_name; // = "/autonomous_driver";
    std::string manual_cmd_topic_name;  // = "/hw/manual_cmd";
    std::string auto_cmd_topic_name;    //   = "/hw/autonomous_cmd";
    std::string emergency_topic_name;   //  = "/hw/emergency_brake";
    std::string cmd_info_topic_name;    //   = "/hw/info";

    int msg_max_period;
    int command_period;
    int info_msg_rate;
    bool use_real_pca9685;
    
    rclcpp::Subscription<maila_msgs::msg::VehicleMode>::SharedPtr subs_mode;
    rclcpp::Subscription<maila_msgs::msg::VehicleControl>::SharedPtr subs_autonomous;
    rclcpp::Subscription<maila_msgs::msg::VehicleControl>::SharedPtr subs_manual;

    rclcpp::TimerBase::SharedPtr timer_subs_mode_timeout;
    rclcpp::TimerBase::SharedPtr timer_subs_autonomous_timeout;
    rclcpp::TimerBase::SharedPtr timer_subs_manual_timeout;
    rclcpp::TimerBase::SharedPtr timer_subs_embrake_timeout;
    rclcpp::TimerBase::SharedPtr timer_command;

    rclcpp::Publisher<maila_msgs::msg::VehicleControl>::SharedPtr pub_info;
    
    std::atomic<bool> subs_mode_ok;
    std::atomic<bool> subs_autonomous_ok;
    std::atomic<bool> subs_manual_ok;

    std::atomic<int> mode;
    std::atomic<float> manual_throttle;
    std::atomic<float> manual_steering_angle;
    std::atomic<float> autonomous_throttle;
    std::atomic<float> autonomous_steering_angle;   

    std::atomic<int> count_command_send; 

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

    void subs_mode_callback(const maila_msgs::msg::VehicleMode::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "subs_mode_callback");
      subs_mode_ok = true;
      mode = msg->mode;

      start_timer_subs_mode_timeout();
    }

    void subs_autonomous_callback(const maila_msgs::msg::VehicleControl::SharedPtr msg)
    {
      RCLCPP_INFO(this->get_logger(), "subs_autonomous_callback");
      subs_autonomous_ok = true;
      autonomous_throttle = msg->throttle;
      autonomous_steering_angle = msg->steering_angle;

      start_timer_subs_autonomous_timeout();
    }

    void subs_manual_callback(const maila_msgs::msg::VehicleControl::SharedPtr msg)
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
      throttle = motors.setThrottle(throttle);
      steering_angle = motors.setSteering(steering_angle);

      // out msg
      count_command_send++;
      if (count_command_send >= info_msg_rate) {
        count_command_send = 0;

        maila_msgs::msg::VehicleControl vehicle_control_msg;
        vehicle_control_msg.throttle = throttle;
        vehicle_control_msg.steering_angle = steering_angle;
        pub_info->publish(vehicle_control_msg);
        RCLCPP_INFO(this->get_logger(), "Publish!");
      
      }


    }

    void create_timers() {

      start_timer_subs_mode_timeout();
      start_timer_subs_autonomous_timeout();
      start_timer_subs_manual_timeout();

      start_timer_command();
    }

    void start_timer_subs_mode_timeout() {
      std::chrono::duration<int, std::milli> timeout_time(msg_max_period);
      timer_subs_mode_timeout = this->create_wall_timer(
        timeout_time, std::bind(&MotorsInterface::timer_subs_mode_timeout_callback, this)
        );
    }

    void start_timer_subs_autonomous_timeout() {
      std::chrono::duration<int, std::milli> timeout_time(msg_max_period);
      timer_subs_autonomous_timeout = this->create_wall_timer(
        timeout_time, std::bind(&MotorsInterface::timer_subs_autonomous_timeout_callback, this)
        );
    }

    void start_timer_subs_manual_timeout() {
      std::chrono::duration<int, std::milli> timeout_time(msg_max_period);
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
      subs_mode = this->create_subscription<maila_msgs::msg::VehicleMode>(
        mode_topic_name, 10, std::bind(&MotorsInterface::subs_mode_callback, this, _1));

      subs_autonomous = this->create_subscription<maila_msgs::msg::VehicleControl>(
        auto_cmd_topic_name, 10, std::bind(&MotorsInterface::subs_autonomous_callback, this, _1));

      subs_manual = this->create_subscription<maila_msgs::msg::VehicleControl>(
        manual_cmd_topic_name, 10, std::bind(&MotorsInterface::subs_manual_callback, this, _1));
    }

    void create_publishers() {
      pub_info = this->create_publisher<maila_msgs::msg::VehicleControl>(cmd_info_topic_name, 10);
      
    }

    void load_parameters_and_initialization() {

      float max_steering;
      float steering_pwm_range, steering_pwm_dead_range, steering_pwm_center;

      float max_throttle;
      float throttle_pwm_range, throttle_pwm_dead_range, throttle_pwm_center;

      this->get_parameter("topics.in_mode", mode_topic_name);
      this->get_parameter("topics.in_manual_cmd", manual_cmd_topic_name);
      this->get_parameter("topics.in_auto_cmd", auto_cmd_topic_name);
      this->get_parameter("topics.in_emergency", emergency_topic_name);
      this->get_parameter("topics.out_cmd_info", cmd_info_topic_name);
      this->get_parameter("in_msg_max_period", msg_max_period);
      this->get_parameter("command_period", command_period);
      this->get_parameter("out_msg_rate", info_msg_rate);
      this->get_parameter("use_real_pca9685", use_real_pca9685);
      this->get_parameter("max_steering", max_steering);
      this->get_parameter("steering_pwm.center", steering_pwm_center);
      this->get_parameter("steering_pwm.dead_range", steering_pwm_dead_range);
      this->get_parameter("steering_pwm.range", steering_pwm_range);
      this->get_parameter("max_throttle", max_throttle);
      this->get_parameter("throttle_pwm.center", throttle_pwm_center);
      this->get_parameter("throttle_pwm.dead_range", throttle_pwm_dead_range);
      this->get_parameter("throttle_pwm.range", throttle_pwm_range);

      if (!use_real_pca9685) {
        RCLCPP_WARN(this->get_logger(), "Real PWM disabled from yaml");
      }

      if (!motors.setupPCA9685(use_real_pca9685)) {
        RCLCPP_ERROR(this->get_logger(), "PCA9685 NOT SETUP");
      }

      motors.setSteeringParams(
        max_steering,
        steering_pwm_range,
        steering_pwm_dead_range,
        steering_pwm_center
      );

      motors.setThrottleParams(
        max_throttle,
        throttle_pwm_range,
        throttle_pwm_dead_range,
        throttle_pwm_center
      );

    }

    void declare_parameters() {
      this->declare_parameter<std::string>("topics.in_mode", "");
      this->declare_parameter<std::string>("topics.in_manual_cmd", "");
      this->declare_parameter<std::string>("topics.in_auto_cmd", "");
      this->declare_parameter<std::string>("topics.in_emergency", "");
      this->declare_parameter<std::string>("topics.out_cmd_info", "");

      this->declare_parameter<int>("in_msg_max_period", 1000);
      this->declare_parameter<int>("command_period", 100);
      this->declare_parameter<int>("out_msg_rate", 100);
      this->declare_parameter<bool>("use_real_pca9685", false);

      this->declare_parameter<float>("max_steering", 1);
      this->declare_parameter<float>("steering_pwm.center", 0);
      this->declare_parameter<float>("steering_pwm.dead_range", 0);
      this->declare_parameter<float>("steering_pwm.range", 1);

      this->declare_parameter<float>("max_throttle", 1);
      this->declare_parameter<float>("throttle_pwm.center", 0);
      this->declare_parameter<float>("throttle_pwm.dead_range", 0);
      this->declare_parameter<float>("throttle_pwm.range", 1);
    }
 
    
    

  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorsInterface>());
    rclcpp::shutdown();
    return 0;
  }
