#ifndef PCA9685MOTORS_H
#define PCA9685MOTORS_H
 
#include "motors_interface/pwm_motor.h"
#include "motors_interface/pca9685.h"
#include <wiringPi.h>

#define PCA9685_ADDRESS 0x40
#define PCA9685_STEERING_SX_CH 2
#define PCA9685_STEERING_DX_CH 1
#define PCA9685_THROTTLE_CH 0
#define PCA9685_PIN_BASE 300
#define PCA9685_MAX_PWM 4096
#define PCA9685_HERTZ 50

class PCA9685Motors
{

private:
    PWMMotor *m_steering_pwm, *m_throttle_pwm;

    float m_max_steering;
    float m_max_throttle;
    bool m_real_hardware;

    float trim(float value);

    void setPwmMotorParams(
        PWMMotor **motor,
        float output_half_range,
        float output_half_dead_range,
        float output_center_value);
 
public:
    PCA9685Motors();

    bool setupPCA9685(bool real_hardware = true);
 
    void setSteeringParams(
        float input_max_value,
        float output_half_range,
        float output_half_dead_range,
        float output_center_value);

    void setThrottleParams(
        float input_max_value,
        float output_half_range,
        float output_half_dead_range,
        float output_center_value);


    float setThrottle(float throttle);
    float setSteering(float steering);
    
    
};
 
#endif