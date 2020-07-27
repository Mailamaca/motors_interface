#include <chrono>
#include <functional>
#include <memory>
#include <string>

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
      
      
      publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
      timer_ = this->create_wall_timer(
        500ms, std::bind(&MotorsInterface::timer_callback, this)
        );

      subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MotorsInterface::topic_callback, this, _1));
    }

    void load_parameters() {
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

  private:
    void timer_callback()
    {
      this->get_parameter("ciao", parameter_string_);

      RCLCPP_INFO(this->get_logger(), "Hello %s", parameter_string_.c_str());

      auto message = std_msgs::msg::String();
      
      message.data = "Hello, world! " + std::to_string(count_++);
      
      RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      
      publisher_->publish(message);
    }

    void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
    }

    /**
    * input is [-1..1]
    * output is [min..max]
    */
    float map(float input, float min, float max, float dead_range){
      const float EPS = 1e-4;
      float out = (min + max)/2.0; // Center
      if(input > EPS){
        const float center_tmp = (out + dead_range);
        out = center_tmp + (max - center_tmp)*input;
      }
      if(input < -EPS){
        const float center_tmp = (out - dead_range);
        out = center_tmp - (min - center_tmp)*input;
      }
      return out;
    }

    float trimValue(float value){
      if(value >  1.) value =  1.;
      if(value < -1.) value = -1.;
      return value;	
    }

    std::string parameter_string_;

    // Topic name list
    std::string manual_cmd_topic_name;  // = "/hw/manual_cmd";
    std::string auto_cmd_topic_name;    //   = "/hw/autonomous_cmd";
    std::string cmd_info_topic_name;    //   = "/hw/info";
    std::string emergency_topic_name;   //  = "/hw/emergency_brake";
    std::string autonomous_mode_topic_name; // = "/autonomous_driver";
    std::string cmd_sim_topic_name;     //      = "/sim/cmd";

    float max_steering;
    float steer_range, steer_dead_range, steer_center;

    float max_speed;
    float motor_range, motor_dead_range, motor_center;

    bool sim_publisher = false;
    double min_subs_period;
    
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    size_t count_;

  };

  int main(int argc, char * argv[])
  {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorsInterface>());
    rclcpp::shutdown();
    return 0;
  }
