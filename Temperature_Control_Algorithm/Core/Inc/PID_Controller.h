/*
 * @file PID_Controller.h
 * Author: TRAN NGUYEN HIEN
 * Email: trannguyenhien29085@gmail.com
 */
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

#ifndef PID_GET_TICK
  #define PID_GET_TICK() HAL_GetTick()
#endif

// Constants used in some of the functions below
#define PID_AUTOMATIC    1
#define PID_MANUAL       0
#define PID_DIRECT       0
#define PID_REVERSE      1
#define PID_P_ON_M       0
#define PID_P_ON_E       1

// Data and state storage structure of the PID controller
typedef struct {
	/* The Kp, Ki, and Kd parameters are displayed */
    float dispKp;
    float dispKi;
    float dispKd;
    /* The parameters Kp, Ki, and Kd have been recalculated according to SampleTime */
    float kp;
    float ki;
    float kd;

    uint8_t controllerDirection;
    uint8_t pOn;
    bool pOnE;
    bool inAuto;

    float *myInput;      // The pointer points to the Input variable
    float *myOutput;     // The pointer points to the Output variable
    float *mySetpoint;   // The pointer points to the Setpoint variable

    uint32_t lastTime;
    float outputSum;
    float lastInput;

    uint32_t SampleTime;
    float outMin;
    float outMax;
} PID_TypeDef;

/* ==================================================================== */
/* CONSTRUCTOR AND CALCULATION FUNCTIONS                                */
/* ==================================================================== */

// Initialize the PID controller with all parameters
void PID_Init(PID_TypeDef *pid, float *Input, float *Output, float *Setpoint,
              float Kp, float Ki, float Kd, uint8_t POn, uint8_t ControllerDirection);

// Initialize the PID controller with the default configuration (P_ON_E)
void PID_Init_Standard(PID_TypeDef *pid, float *Input, float *Output, float *Setpoint,
                       float Kp, float Ki, float Kd, uint8_t ControllerDirection);

// The core calculation function is called repeatedly within the main loop
bool PID_Compute(PID_TypeDef *pid);

/* ==================================================================== */
/* CONFIGURATION FUNCTIONS                                              */
/* ==================================================================== */

// Switch between automatic (PID_AUTOMATIC) and manual (PID_MANUAL) modes
void PID_SetMode(PID_TypeDef *pid, uint8_t Mode);

// Output value limit
void PID_SetOutputLimits(PID_TypeDef *pid, float Min, float Max);

// Change the Kp, Ki, and Kd parameters during runtime
void PID_SetTunings(PID_TypeDef *pid, float Kp, float Ki, float Kd, uint8_t POn);
void PID_SetTunings_Standard(PID_TypeDef *pid, float Kp, float Ki, float Kd);

// Set the control direction: Forward (DIRECT) or Reverse (REVERSE)
void PID_SetControllerDirection(PID_TypeDef *pid, uint8_t Direction);

// Set the sampling time (in milliseconds).
void PID_SetSampleTime(PID_TypeDef *pid, uint32_t NewSampleTime);

/* ==================================================================== */
/* INFORMATION EXTRACTION FUNCTIONS                                     */
/* ==================================================================== */
float PID_GetKp(PID_TypeDef *pid);
float PID_GetKi(PID_TypeDef *pid);
float PID_GetKd(PID_TypeDef *pid);
uint8_t PID_GetMode(PID_TypeDef *pid);
uint8_t PID_GetDirection(PID_TypeDef *pid);

#endif // PID_CONTROLLER_H
