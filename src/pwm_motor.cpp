#include "motors_interface/pwm_motor.h"

/**
 * @brief Construct a new PWMMotor::PWMMotor object
 * 
 * @param output_half_range 
 * @param output_half_dead_range 
 * @param output_center_value 
 * @param max_pwm_value 
 */
PWMMotor::PWMMotor(float output_half_range,
    float output_half_dead_range,
    float output_center_value,
    int max_pwm_value) :
        m_output_half_range(output_half_range),
        m_output_half_dead_range(output_half_dead_range),
        m_output_center_value(output_center_value),
        m_max_pwm_value(max_pwm_value)
{
    
}

/**
 * @brief Map a value between [-1,1] to the range [min,max] considering a dead range near to center of the output
 * 
 * @param input value between [-1,1]
 * @param min exit min value
 * @param max exit max value
 * @param half_dead_range 0 exit near the center of output
 * @return float 
 */
float PWMMotor::map(float input, float min, float max, float half_dead_range){
    float EPS = 1e-4;
    float out = (min + max)/2.0; // Center
    if(input > EPS){ 
        float center_tmp = (out + half_dead_range);
        out = center_tmp + (max - center_tmp)*input;
    }
    else if(input < -EPS){
        float center_tmp = (out - half_dead_range);
        out = center_tmp - (min - center_tmp)*input;
    }
    return out;
}

/**
 * @brief Calculate the value to send to the PWM motor in the range [0,max_pwm_value]
 * 
 * @param value number in the range [-input_max_value,input_max_value]
 * @return int value to send to the PWM motor in the range [0,max_pwm_value]
 */
int PWMMotor::calculate(float value){

    float duty_cycle;

    duty_cycle = map(value,
                            m_output_center_value - m_output_half_range, 
							m_output_center_value + m_output_half_range,
                            m_output_half_dead_range);
	
	return (int)(m_max_pwm_value * duty_cycle + 0.5f);

}