#include "motors_interface/pca9685_motors.h"
 
// Constructor
PCA9685Motors::PCA9685Motors()
{
    
}

void PCA9685Motors::SetPCA9685Params(
        int i2c_address,
        int pin_base,
        int hertz,
        int speed_channel,
        int steer_dx_channel,
        int steer_sx_channel)           
{
    m_i2c_address = i2c_address;
    m_pin_base = pin_base;
    m_hertz = hertz;
    m_speed_channel = speed_channel;
    m_steer_dx_channel = steer_dx_channel;
    m_steer_sx_channel = steer_sx_channel;
}
 
void PCA9685Motors::SetPwmMotorParams(
        PWMMotor **motor,
        float input_max_value,
        float output_half_range,
        float output_half_dead_range,
        float output_center_value,
        int max_pwm_value)
{
    *motor = new PWMMotor(
        input_max_value,
        output_half_range,
        output_half_dead_range,
        output_center_value,
        max_pwm_value
    );
}

void PCA9685Motors::SetSteerMotorParams(
        float input_max_value,
        float output_half_range,
        float output_half_dead_range,
        float output_center_value,
        int max_pwm_value)
{
    SetPwmMotorParams(
        &m_steer,
        input_max_value,
        output_half_range,
        output_half_dead_range,
        output_center_value,
        max_pwm_value
    );
}

void PCA9685Motors::SetSpeedMotorParams(
        float input_max_value,
        float output_half_range,
        float output_half_dead_range,
        float output_center_value,
        int max_pwm_value)
{
    SetPwmMotorParams(
        &m_speed,
        input_max_value,
        output_half_range,
        output_half_dead_range,
        output_center_value,
        max_pwm_value
    );
}