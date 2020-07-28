#ifndef PCA9685MOTORS_H
#define PCA9685MOTORS_H
 
#include "motors_interface/pwm_motor.h"

class PCA9685Motors
{

private:
    int m_steer_sx_channel;
    int m_steer_dx_channel;
    int m_speed_channel;
    int m_pin_base;
    int m_hertz;
    int m_i2c_address;
    PWMMotor *m_steer, *m_speed;

    void SetPwmMotorParams(
        PWMMotor **motor,
        float input_max_value,
        float output_half_range,
        float output_half_dead_range,
        float output_center_value,
        int max_pwm_value);
 
public:
    PCA9685Motors();

    void SetPCA9685Params(
        int i2c_address,
        int pin_base,
        int hertz,
        int speed_channel,
        int steer_dx_channel,
        int steer_sx_channel);
 
    void SetSteerMotorParams(
        float input_max_value,
        float output_half_range,
        float output_half_dead_range,
        float output_center_value,
        int max_pwm_value);

    void SetSpeedMotorParams(
        float input_max_value,
        float output_half_range,
        float output_half_dead_range,
        float output_center_value,
        int max_pwm_value);
 
    
    
};
 
#endif