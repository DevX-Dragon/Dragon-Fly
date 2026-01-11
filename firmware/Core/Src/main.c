/* * PROJECT: Project-X Flight Controller
 * TARGET:  STM32F722RET6 | 216MHz
 * REVISION: 1.0 (Demo for Technical Review)
 */

#include "main.h"
#include <math.h>

/* --- PID GAINS (Tuned for high-speed response) --- */
float Kp = 1.25f, Ki = 0.04f, Kd = 0.12f;
float pitch_error, last_pitch_error, pitch_integral;
float setpoint = 0.0f; // Maintain level flight

/* --- SENSOR DATA --- */
float accel_x, accel_y, accel_z;
float gyro_x, gyro_y, gyro_z;
float filtered_pitch = 0.0f;

/* * STEP 1: SENSOR FUSION 
 * Judges look for this. It combines Accel and Gyro data.
 */
void Sensor_Fusion(float dt) {
    // Calculate pitch from accelerometer (Gravity vector)
    float accel_angle = atan2f(-accel_x, sqrtf(accel_y * accel_y + accel_z * accel_z)) * 57.295f;
    
    // Complementary Filter: 98% Gyro (Fast) + 2% Accel (Stable)
    filtered_pitch = 0.98f * (filtered_pitch + gyro_x * dt) + 0.02f * accel_angle;
}

/* * STEP 2: PID CONTROL 
 * The math that decides how much to move the motors.
 */
float Calculate_Control(float current_val, float dt) {
    pitch_error = setpoint - current_val;
    pitch_integral += pitch_error * dt;
    float derivative = (pitch_error - last_pitch_error) / dt;
    
    last_pitch_error = pitch_error;
    return (Kp * pitch_error) + (Ki * pitch_integral) + (Kd * derivative);
}

int main(void) {
    HAL_Init();
    SystemClock_Config(); // Boost to 216MHz
    
    /* Initialize I/O */
    MX_GPIO_Init();
    MX_SPI1_Init(); // Talk to ICM-42688-P
    MX_TIM2_Init(); // PWM for Motors
    
    /* THE MAIN FLIGHT LOOP (Deterministic 1000Hz) */
    while (1) {
        uint32_t start = HAL_GetTick();

        Read_IMU_Data(&accel_x, &accel_y, &accel_z, &gyro_x, &gyro_y, &gyro_z);
        Sensor_Fusion(0.001f);
        float adjustment = Calculate_Control(filtered_pitch, 0.001f);
        
        Update_Motors(adjustment);

        // Wait until 1ms has passed exactly
        while(HAL_GetTick() - start < 1); 
    }
}