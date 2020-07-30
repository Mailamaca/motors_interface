#include "motors_interface/pca9685_motors.h"
 
// Constructor
PCA9685Motors::PCA9685Motors()
{
    
}

bool PCA9685Motors::setupPCA9685(bool real_hardware)          
{
    m_real_hardware = real_hardware;

    if (m_real_hardware) {
        // Calling wiringPi setup first.
        wiringPiSetup();

        // Setup with pinbase 300 and i2c location 0x40
        int fd0 = pca9685Setup(PCA9685_PIN_BASE , PCA9685_ADDRESS, PCA9685_HERTZ);
        if (fd0 < 0)
        {
            return false;
        }    
        pca9685PWMReset(fd0);
    }
    return true;
}
 
void PCA9685Motors::setPwmMotorParams(
        PWMMotor **motor,
        float output_half_range,
        float output_half_dead_range,
        float output_center_value)
{
    *motor = new PWMMotor(
        output_half_range,
        output_half_dead_range,
        output_center_value,
        PCA9685_MAX_PWM
    );
}

/**
 * @brief trim value between -1 and 1
 * 
 * @param value 
 * @return float value trimmed [-1,1]
 */
float PCA9685Motors::trim(float value){
	if(value >  1.) {
        return 1.0;
    }
	else if(value < -1.) {
        return -1.0;
    }
    else {
        return value;
    }
}

void PCA9685Motors::setSteeringParams(
        float input_max_value,
        float output_half_range,
        float output_half_dead_range,
        float output_center_value)
{
    m_max_steering = input_max_value;
    setPwmMotorParams(
        &m_steering_pwm,
        output_half_range,
        output_half_dead_range,
        output_center_value);
}

void PCA9685Motors::setThrottleParams(
        float input_max_value,
        float output_half_range,
        float output_half_dead_range,
        float output_center_value)
{
    m_max_throttle = input_max_value;
    setPwmMotorParams(
        &m_throttle_pwm,
        output_half_range,
        output_half_dead_range,
        output_center_value);
}

float PCA9685Motors::setThrottle(float throttle)
{
    throttle = trim(throttle/m_max_throttle);
    int th = m_throttle_pwm->calculate(throttle);
    if (m_real_hardware) {
        pwmWrite(PCA9685_PIN_BASE + PCA9685_THROTTLE_CH, th);
    }

    return throttle;
}

float PCA9685Motors::setSteering(float steering)
{
    steering = trim(steering/m_max_steering);
    int st = m_steering_pwm->calculate(steering);
    if (m_real_hardware) {
        pwmWrite(PCA9685_PIN_BASE + PCA9685_STEERING_DX_CH, st);
        pwmWrite(PCA9685_PIN_BASE + PCA9685_STEERING_SX_CH, st);
    }

    return steering;
}