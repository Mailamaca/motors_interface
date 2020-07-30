#ifndef PWMMOTOR_H
#define PWMMOTOR_H

class PWMMotor
{

private:

    float m_input_max_value;
    float m_output_half_range;
    float m_output_half_dead_range;
    float m_output_center_value;

    int m_max_pwm_value;

    float map(float input, float min, float max, float half_dead_range);
 
public:
    PWMMotor(
        float output_half_range,
        float output_half_dead_range,
        float output_center_value,
        int max_pwm_value); 

    int calculate(float value);
    
};
 
#endif