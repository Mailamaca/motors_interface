#include "motors_interface/motor_component.hpp"

#include <cinttypes>
#include <iostream>
#include <cstring>
#include <memory>

extern "C" {
  #include <stdio.h>
  #include <stdlib.h>
  #include <fcntl.h>
  #include <errno.h>
  #include <unistd.h>
}

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace motor
{

MotorComponent::MotorComponent(const rclcpp::NodeOptions & options)
: Node("MotorComponent", options) {
  RCLCPP_INFO( this->get_logger(), "MotorComponent::MotorComponent");

  load_parameters();    
  
  create_publishers();
  create_subscriptions();
  create_clients();
  
   

  initialize();

  //client_read = create_client<i2c_interfaces::srv::I2cRead>("i2c_read");
  //timer_ = create_wall_timer(10ms, std::bind(&I2CClient::on_timer, this));
 
}

void MotorComponent::load_parameters() {

  this->declare_parameter<std::string>("topics.in_mode", "a");
  this->get_parameter("topics.in_mode", mode_topic_name);
  //RCLCPP_INFO( this->get_logger(), "topics.in_mode: %s", mode_topic_name.c_str());

  this->declare_parameter<std::string>("topics.in_manual_cmd", "b");
  this->get_parameter("topics.in_manual_cmd", manual_cmd_topic_name);

  this->declare_parameter<std::string>("topics.in_auto_cmd", "c");
  this->get_parameter("topics.in_auto_cmd", auto_cmd_topic_name);

  this->declare_parameter<std::string>("topics.in_emergency", "d");
  this->get_parameter("topics.in_emergency", emergency_topic_name);

  this->declare_parameter<std::string>("topics.out_cmd_info", "e");
  this->get_parameter("topics.out_cmd_info", cmd_info_topic_name);

  this->declare_parameter<int>("in_msg_max_period", 1000);
  this->get_parameter("in_msg_max_period", msg_max_period);

  this->declare_parameter<int>("command_period", 10000);
  this->get_parameter("command_period", command_period);

  this->declare_parameter<int>("out_msg_rate", 100);
  this->get_parameter("out_msg_rate", info_msg_rate);

  this->declare_parameter<bool>("use_real_pca9685", true);
  this->get_parameter("use_real_pca9685", use_real_pca9685);

  this->declare_parameter<float>("max_steering", 1);
  this->get_parameter("max_steering", max_steering);

  this->declare_parameter<float>("steering_pwm.center", 0);
  this->get_parameter("steering_pwm.center", steering_pwm_center);

  this->declare_parameter<float>("steering_pwm.dead_range", 0);
  this->get_parameter("steering_pwm.dead_range", steering_pwm_dead_range);

  this->declare_parameter<float>("steering_pwm.range", 1);
  this->get_parameter("steering_pwm.range", steering_pwm_range);

  this->declare_parameter<int>("steering_pwm.max_pwm", 4096);
  this->get_parameter("steering_pwm.max_pwm", steering_max_pwm);

  this->declare_parameter<float>("max_throttle", 1);
  this->get_parameter("max_throttle", max_throttle);

  this->declare_parameter<float>("throttle_pwm.center", 0);
  this->get_parameter("throttle_pwm.center", throttle_pwm_center);

  this->declare_parameter<float>("throttle_pwm.dead_range", 0);
  this->get_parameter("throttle_pwm.dead_range", throttle_pwm_dead_range);

  this->declare_parameter<float>("throttle_pwm.range", 1);
  this->get_parameter("throttle_pwm.range", throttle_pwm_range);

  this->declare_parameter<int>("throttle_pwm.max_pwm", 4096);
  this->get_parameter("throttle_pwm.max_pwm", throttle_max_pwm);
           

}

void MotorComponent::create_subscriptions() {
  subs_mode = this->create_subscription<maila_msgs::msg::VehicleMode>(
    mode_topic_name, 10, std::bind(&MotorComponent::subs_mode_callback, this, _1));
  this->start_timer_subs_mode_timeout();

  subs_autonomous = this->create_subscription<maila_msgs::msg::VehicleControl>(
    auto_cmd_topic_name, 10, std::bind(&MotorComponent::subs_autonomous_callback, this, _1));
  this->start_timer_subs_autonomous_timeout();

  subs_manual = this->create_subscription<maila_msgs::msg::VehicleControl>(
    manual_cmd_topic_name, 10, std::bind(&MotorComponent::subs_manual_callback, this, _1));
  this->start_timer_subs_manual_timeout();

}

void MotorComponent::create_publishers() {
  pub_info = this->create_publisher<maila_msgs::msg::VehicleControl>(cmd_info_topic_name, 10);
  
}

void MotorComponent::create_clients() {
  i2c_command = this->create_client<i2c_interfaces::srv::I2cCommand>("i2c_command");

}

void MotorComponent::initialize() {
  
  if (!use_real_pca9685) {
    RCLCPP_WARN(this->get_logger(), "Real PWM disabled from yaml");
    pca9685_initialized = true;
  } else {
    this->init_pca9685_step1();
  }

  this->steering_pwm.setParams(
    max_steering,
    steering_pwm_range,
    steering_pwm_dead_range,
    steering_pwm_center,
    steering_max_pwm
  );

  this->throttle_pwm.setParams(
    max_throttle,
    throttle_pwm_range,
    throttle_pwm_dead_range,
    throttle_pwm_center,
    throttle_max_pwm
  );

  RCLCPP_INFO(this->get_logger(), "throttle_pwm->\t \
                                  max_throttle:%f\t \
                                  throttle_pwm_range:%f\t \
                                  throttle_pwm_dead_range:%f\t \
                                  throttle_pwm_center:%f ",
                                  max_throttle,
                                  throttle_pwm_range,
                                  throttle_pwm_dead_range,
                                  throttle_pwm_center);
  


}

bool MotorComponent::call_i2c_command_service(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Request> request, std::function<void(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response>)> cb) {

  //RCLCPP_INFO( this->get_logger(), "call_i2c_command_service");

  if (!i2c_command->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Interrupted while waiting for 'i2c_command' service. Exiting.");
      return false;
    }
    RCLCPP_WARN(
      this->get_logger(),
      "Service 'i2c_command' not available after waiting");
    return false;
  }

  using ServiceResponseFuture =
    rclcpp::Client<i2c_interfaces::srv::I2cCommand>::SharedFuture;
  auto response_received_callback = [/*this, */cb](ServiceResponseFuture future) {
    //RCLCPP_INFO( this->get_logger(), "response_received_callback");
    cb(future.get());
  };

  //RCLCPP_INFO( this->get_logger(), "async_send_request");
  auto future_result = i2c_command->async_send_request(request, response_received_callback);
  
  return true;
}

void MotorComponent::init_pca9685_step1() {

  // wiringPiI2CReadReg8(fd, PCA9685_MODE1)

  RCLCPP_INFO( this->get_logger(), "init_pca9685_step1");

  auto request = std::make_shared<i2c_interfaces::srv::I2cCommand::Request>();
  request->slave = PCA9685_ADDRESS;
  request->reg = PCA9685_MODE1;
  request->write = false;
  request->length = 1;

  auto fp = std::bind(&MotorComponent::init_pca9685_step2, this, _1);
  call_i2c_command_service(request, fp);  
}

void MotorComponent::init_pca9685_step2(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response> response) {

  //int settings = wiringPiI2CReadReg8(fd, PCA9685_MODE1) & 0x7F;
	//int autoInc = settings | 0x20;
	//wiringPiI2CWriteReg8(fd, PCA9685_MODE1, autoInc);

  RCLCPP_INFO( this->get_logger(), "init_pca9685_step2");

  // evaluate response
  if (!response->ok) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Error on pca9685 1° communication");
    return;
  }
  this->pca9685_settings = response->data_received.at(0) & 0x7F;
  this->pca9685_autoInc = this->pca9685_settings | 0x20;
  this->pca9685_sleep	= this->pca9685_autoInc | 0x10;				// Set sleep bit to 1
	this->pca9685_wake 	= this->pca9685_autoInc & 0xEF;				// Set sleep bit to 0
	this->pca9685_restart = this->pca9685_wake | 0x80;

  
  auto request = std::make_shared<i2c_interfaces::srv::I2cCommand::Request>();
  request->slave = PCA9685_ADDRESS;
  request->reg = PCA9685_MODE1;
  request->write = true;
  request->length = 1;
  std::vector<uint8_t> vectData;
  vectData.push_back((uint8_t) this->pca9685_autoInc);
  request->data_to_send = vectData;

  auto fp = std::bind(&MotorComponent::init_pca9685_step3, this, _1);
  call_i2c_command_service(request, fp);  
  
}

void MotorComponent::init_pca9685_step3(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response> response) {
  
  //freq = (freq > 1000 ? 1000 : (freq < 40 ? 40 : freq));
  //int prescale = (int)(25000000.0f / (4096 * freq) - 0.5f);
  //wiringPiI2CWriteReg8(fd, PCA9685_MODE1, sleep);

  RCLCPP_INFO( this->get_logger(), "init_pca9685_step3");

  // evaluate response
  if (!response->ok) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Error on pca9685 2° communication");
    return;
  }
  int freq = PCA9685_HERTZ;
  freq = (freq > 1000 ? 1000 : (freq < 40 ? 40 : freq));
  this->pca9685_prescale = (int)(25000000.0f / (4096 * freq) - 0.5f);

  auto request = std::make_shared<i2c_interfaces::srv::I2cCommand::Request>();
  request->slave = PCA9685_ADDRESS;
  request->reg = PCA9685_MODE1;
  request->write = true;
  request->length = 1;
  std::vector<uint8_t> vectData;
  vectData.push_back((uint8_t) this->pca9685_sleep);
  request->data_to_send = vectData;

  auto fp = std::bind(&MotorComponent::init_pca9685_step4, this, _1);
  call_i2c_command_service(request, fp); 
  
}

void MotorComponent::init_pca9685_step4(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response> response) {
  
  //wiringPiI2CWriteReg8(fd, PCA9685_PRESCALE, prescale);

  RCLCPP_INFO( this->get_logger(), "init_pca9685_step4");

  // evaluate response
  if (!response->ok) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Error on pca9685 3° communication");
    return;
  }
  
  auto request = std::make_shared<i2c_interfaces::srv::I2cCommand::Request>();
  request->slave = PCA9685_ADDRESS;
  request->reg = PCA9685_PRESCALE;
  request->write = true;
  request->length = 1;
  std::vector<uint8_t> vectData;
  vectData.push_back((uint8_t) this->pca9685_prescale);
  request->data_to_send = vectData;

  auto fp = std::bind(&MotorComponent::init_pca9685_step5, this, _1);
  call_i2c_command_service(request, fp); 
  
}

void MotorComponent::init_pca9685_step5(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response> response) {
  
  //wiringPiI2CWriteReg8(fd, PCA9685_MODE1, wake);

  RCLCPP_INFO( this->get_logger(), "init_pca9685_step5");
  

  // evaluate response
  if (!response->ok) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Error on pca9685 4° communication");
    return;
  }
  
  auto request = std::make_shared<i2c_interfaces::srv::I2cCommand::Request>();
  request->slave = PCA9685_ADDRESS;
  request->reg = PCA9685_MODE1;
  request->write = true;
  request->length = 1;
  std::vector<uint8_t> vectData;
  vectData.push_back((uint8_t) this->pca9685_wake);
  request->data_to_send = vectData;

  auto fp = std::bind(&MotorComponent::init_pca9685_step6, this, _1);
  call_i2c_command_service(request, fp); 
  
}

void MotorComponent::init_pca9685_step6(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response> response) {
  
  //wiringPiI2CWriteReg8(fd, PCA9685_MODE1, restart);

  RCLCPP_INFO( this->get_logger(), "init_pca9685_step6");

  std::this_thread::sleep_for(std::chrono::milliseconds(1));

  // evaluate response
  if (!response->ok) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Error on pca9685 5° communication");
    return;
  }
  
  auto request = std::make_shared<i2c_interfaces::srv::I2cCommand::Request>();
  request->slave = PCA9685_ADDRESS;
  request->reg = PCA9685_MODE1;
  request->write = true;
  request->length = 1;
  std::vector<uint8_t> vectData;
  vectData.push_back((uint8_t) this->pca9685_restart);
  request->data_to_send = vectData;

  auto fp = std::bind(&MotorComponent::init_pca9685_step7, this, _1);
  call_i2c_command_service(request, fp); 
  
}

void MotorComponent::init_pca9685_step7(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response> response) {
  
  //wiringPiI2CWriteReg16(fd, LEDALL_ON_L	 , 0x0);

  RCLCPP_INFO( this->get_logger(), "init_pca9685_step7");

  // evaluate response
  if (!response->ok) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Error on pca9685 6° communication");
    return;
  }
  
  auto request = std::make_shared<i2c_interfaces::srv::I2cCommand::Request>();
  request->slave = PCA9685_ADDRESS;
  request->reg = PCA9685_LEDALL_ON_L;
  request->write = true;
  request->length = 2;
  std::vector<uint8_t> vectData;
  vectData.push_back(0x00);
  vectData.push_back(0x00);
  request->data_to_send = vectData;

  auto fp = std::bind(&MotorComponent::init_pca9685_step8, this, _1);
  call_i2c_command_service(request, fp); 
  
}

void MotorComponent::init_pca9685_step8(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response> response) {
  
  //wiringPiI2CWriteReg16(fd, LEDALL_ON_L+2	 , 0x1000);

  RCLCPP_INFO( this->get_logger(), "init_pca9685_step8");

  // evaluate response
  if (!response->ok) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Error on pca9685 7° communication");
    return;
  }
  
  auto request = std::make_shared<i2c_interfaces::srv::I2cCommand::Request>();
  request->slave = PCA9685_ADDRESS;
  request->reg = PCA9685_LEDALL_ON_L+2;
  request->write = true;
  request->length = 2;
  std::vector<uint8_t> vectData;
  vectData.push_back(0x10);
  vectData.push_back(0x00);
  request->data_to_send = vectData;

  auto fp = std::bind(&MotorComponent::init_pca9685_step9, this, _1);
  call_i2c_command_service(request, fp); 
  
}

void MotorComponent::init_pca9685_step9(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response> response) {
  
  RCLCPP_INFO( this->get_logger(), "init_pca9685_step9");

  // evaluate response
  if (!response->ok) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Error on pca9685 8° communication");
    return;
  }
  
  this->pca9685_initialized = true;
  RCLCPP_INFO( this->get_logger(), "pca9685_initialized!");

  this->start_timer_command();
  
}

void MotorComponent::send_pwm(uint8_t reg, int value) {

  //wiringPiI2CWriteReg16(fd, LEDALL_ON_L+2	 , 0x1000);

  //RCLCPP_INFO( this->get_logger(), "send_pwm: reg:%d", reg);
  
  auto request = std::make_shared<i2c_interfaces::srv::I2cCommand::Request>();
  request->slave = PCA9685_ADDRESS;
  request->reg = reg;
  request->write = true;
  request->length = 2;
  std::vector<uint8_t> vectData;

  vectData.push_back((uint8_t)(value >> 0));
  vectData.push_back((uint8_t)(value >> 8));
  request->data_to_send = vectData;

  //RCLCPP_INFO( this->get_logger(), "vectData[0]:%d", vectData.at(0));
  //RCLCPP_INFO( this->get_logger(), "vectData[1]:%d", vectData.at(1));

  auto fp = std::bind(&MotorComponent::pca9685_cmd_cb, this, _1);
  call_i2c_command_service(request, fp); 

}

void MotorComponent::pca9685_cmd_cb(std::shared_ptr<i2c_interfaces::srv::I2cCommand::Response> response) {
  
  //RCLCPP_INFO( this->get_logger(), "pca9685_cmd_cb");

  // evaluate response
  if (!response->ok) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Error on pca9685 pca9685_cmd_cb");
    return;
  }
  
}

void MotorComponent::timer_subs_mode_timeout_callback() {
  //RCLCPP_WARN(this->get_logger(), "Timeout timer_subs_mode");
  this->subs_mode_ok = false;
}

void MotorComponent::timer_subs_autonomous_timeout_callback() {
  //RCLCPP_WARN(this->get_logger(), "Timeout timer_subs_autonomous");
  this->subs_autonomous_ok = false;
}

void MotorComponent::timer_subs_manual_timeout_callback() {
  //RCLCPP_WARN(this->get_logger(), "Timeout timer_subs_manual");
  this->subs_manual_ok = false;
}

void MotorComponent::subs_mode_callback(const maila_msgs::msg::VehicleMode::SharedPtr msg) {
  //RCLCPP_INFO(this->get_logger(), "subs_mode_callback");
  this->subs_mode_ok = true;
  this->mode = msg->mode;

  this->start_timer_subs_mode_timeout();
}

void MotorComponent::subs_autonomous_callback(const maila_msgs::msg::VehicleControl::SharedPtr msg) {
  //RCLCPP_INFO(this->get_logger(), "subs_autonomous_callback");
  this->subs_autonomous_ok = true;
  this->autonomous_throttle = msg->throttle;
  this->autonomous_steering_angle = msg->steering_angle;

  this->start_timer_subs_autonomous_timeout();
}

void MotorComponent::subs_manual_callback(const maila_msgs::msg::VehicleControl::SharedPtr msg) {
  //RCLCPP_INFO(this->get_logger(), "subs_manual_callback");
  this->subs_manual_ok = true;
  this->manual_throttle = msg->throttle;
  this->manual_steering_angle = msg->steering_angle;

  this->start_timer_subs_manual_timeout();
}

void MotorComponent::start_timer_subs_mode_timeout() {
  std::chrono::duration<int, std::milli> timeout_time(msg_max_period);
  this->timer_subs_mode_timeout = this->create_wall_timer(
    timeout_time, std::bind(&MotorComponent::timer_subs_mode_timeout_callback, this)
    );
}

void MotorComponent::start_timer_subs_autonomous_timeout() {
  std::chrono::duration<int, std::milli> timeout_time(msg_max_period);
  this->timer_subs_autonomous_timeout = this->create_wall_timer(
    timeout_time, std::bind(&MotorComponent::timer_subs_autonomous_timeout_callback, this)
    );
}

void MotorComponent::start_timer_subs_manual_timeout() {
  std::chrono::duration<int, std::milli> timeout_time(msg_max_period);
  this->timer_subs_manual_timeout = this->create_wall_timer(
    timeout_time, std::bind(&MotorComponent::timer_subs_manual_timeout_callback, this)
    );
}

void MotorComponent::start_timer_command() {
  std::chrono::duration<int, std::milli> timeout_time(command_period);
  this->timer_command = this->create_wall_timer(
    timeout_time, std::bind(&MotorComponent::timer_command_callback, this)
    );
}

void MotorComponent::timer_command_callback() {
  
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

  //RCLCPP_INFO(this->get_logger(), "Command->\tThrottle:%f\tSteeringAngle:%f", throttle, steering_angle);
  
  
  int th_tick = throttle_pwm.calculate(throttle);
  int st_tick = steering_pwm.calculate(steering_angle);

  //RCLCPP_INFO(this->get_logger(), "Command->\tth_tick:%d", th_tick);
  

  // **** TODO: SEND COMMAND TO I2C!!!
  this->send_pwm((uint8_t) (baseReg(PCA9685_THROTTLE_CH)+2), th_tick);  
  this->send_pwm((uint8_t) (baseReg(PCA9685_STEERING_DX_CH)+2), st_tick);  
  this->send_pwm((uint8_t) (baseReg(PCA9685_STEERING_SX_CH)+2), st_tick);  

  // out msg
  count_command_send++;
  if (count_command_send >= info_msg_rate) {
    count_command_send = 0;

    
    maila_msgs::msg::VehicleControl vehicle_control_msg;
    vehicle_control_msg.throttle = throttle;
    vehicle_control_msg.steering_angle = steering_angle;
    pub_info->publish(vehicle_control_msg);
    //RCLCPP_INFO(this->get_logger(), "Publish!");
  
  }


}

int MotorComponent::baseReg(int pin) {
	return (pin >= PCA9685_PIN_ALL ? PCA9685_LEDALL_ON_L : PCA9685_LED0_ON_L + 4 * pin);
}

}  // namespace motor

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(motor::MotorComponent)

