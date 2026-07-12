/*
 * @file PID_Controller.c
 * Author: TRAN NGUYEN HIEN
 * Email: trannguyenhien29085@gmail.com
 */
#include "PID_Controller.h"
#include "main.h"

/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
static void PID_Initialize(PID_TypeDef *pid)
{
    if (pid == NULL || pid->myOutput == NULL || pid->myInput == NULL) return;

    pid->outputSum = *(pid->myOutput);
    pid->lastInput = *(pid->myInput);

    if(pid->outputSum > pid->outMax) pid->outputSum = pid->outMax;
    else if(pid->outputSum < pid->outMin) pid->outputSum = pid->outMin;
}

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
void PID_Init(PID_TypeDef *pid, float *Input, float *Output, float *Setpoint,
              float Kp, float Ki, float Kd, uint8_t POn, uint8_t ControllerDirection)
{
	if (pid == NULL) return;

    pid->myOutput = Output;
    pid->myInput = Input;
    pid->mySetpoint = Setpoint;
    pid->inAuto = false;

    PID_SetOutputLimits(pid, 0.0f, 255.0f);

    pid->SampleTime = 100; // The default sampling time is 100ms

    PID_SetControllerDirection(pid, ControllerDirection);
    PID_SetTunings(pid, Kp, Ki, Kd, POn);

    pid->lastTime = PID_GET_TICK() - pid->SampleTime;
}

/*Constructor (...)*********************************************************
 *    To allow backwards compatability for v1.1, or for people that just want
 *    to use Proportional on Error without explicitly saying so
 ***************************************************************************/
void PID_Init_Standard(PID_TypeDef *pid, float *Input, float *Output, float *Setpoint,
                       float Kp, float Ki, float Kd, uint8_t ControllerDirection)
{
    PID_Init(pid, Input, Output, Setpoint, Kp, Ki, Kd, PID_P_ON_E, ControllerDirection);
}

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID_Compute(PID_TypeDef *pid)
{
	if(pid == NULL || !pid->inAuto) return false;
	if(pid->myInput == NULL || pid->myOutput == NULL || pid->mySetpoint == NULL) return false;

    uint32_t now = PID_GET_TICK();
    uint32_t timeChange = (now - pid->lastTime);

    if(timeChange >= pid->SampleTime)
    {
        float input_val = *(pid->myInput);
        float setpoint_val = *(pid->mySetpoint);

        /* Compute all the working error variables */
        float error = setpoint_val - input_val;
        float dInput = (input_val - pid->lastInput);
        pid->outputSum += (pid->ki * error);

        /* Add Proportional on Measurement, if P_ON_M is specified */
        if(!pid->pOnE) pid->outputSum -= pid->kp * dInput;

        if(pid->outputSum > pid->outMax) pid->outputSum = pid->outMax;
        else if(pid->outputSum < pid->outMin) pid->outputSum = pid->outMin;

        /* Add Proportional on Error, if P_ON_E is specified */
        float output;
        if(pid->pOnE) output = pid->kp * error;
        else output = 0.0f;

        /* Compute Rest of PID Output */
        output += pid->outputSum - pid->kd * dInput;

        if(output > pid->outMax) output = pid->outMax;
        else if(output < pid->outMin) output = pid->outMin;

        *(pid->myOutput) = output;

        /* Remember some variables for next time */
        pid->lastInput = input_val;
        pid->lastTime = now;

        return true;
    }
    else return false;
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID_SetTunings(PID_TypeDef *pid, float Kp, float Ki, float Kd, uint8_t POn)
{
    if (pid == NULL || Kp < 0.0f || Ki < 0.0f || Kd < 0.0f) return;

    pid->pOn = POn;
    pid->pOnE = (POn == PID_P_ON_E);

    pid->dispKp = Kp;
    pid->dispKi = Ki;
    pid->dispKd = Kd;

    float SampleTimeInSec = ((float)pid->SampleTime) / 1000.0f;
    pid->kp = Kp;
    pid->ki = Ki * SampleTimeInSec;
    pid->kd = Kd / SampleTimeInSec;

    if(pid->controllerDirection == PID_REVERSE)
    {
        pid->kp = (0.0f - pid->kp);
        pid->ki = (0.0f - pid->ki);
        pid->kd = (0.0f - pid->kd);
    }
}

/* SetTunings(...)*************************************************************
 * Set Tunings using the last-rembered POn setting
 ******************************************************************************/
void PID_SetTunings_Standard(PID_TypeDef *pid, float Kp, float Ki, float Kd)
{
	if (pid == NULL) return;
    PID_SetTunings(pid, Kp, Ki, Kd, pid->pOn);
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed
 ******************************************************************************/
void PID_SetSampleTime(PID_TypeDef *pid, uint32_t NewSampleTime)
{
	if (pid == NULL) return;
    if (NewSampleTime > 0)
    {
        float ratio = (float)NewSampleTime / (float)pid->SampleTime;
        pid->ki *= ratio;
        pid->kd /= ratio;
        pid->SampleTime = NewSampleTime;
    }
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
void PID_SetOutputLimits(PID_TypeDef *pid, float Min, float Max)
{
    if(pid == NULL || Min >= Max) return;

    pid->outMin = Min;
    pid->outMax = Max;

    if(pid->inAuto)
    {
        if (pid->myOutput != NULL)
        {
            if(*(pid->myOutput) > pid->outMax) *(pid->myOutput) = pid->outMax;
            else if(*(pid->myOutput) < pid->outMin) *(pid->myOutput) = pid->outMin;
        }

        if(pid->outputSum > pid->outMax) pid->outputSum = pid->outMax;
        else if(pid->outputSum < pid->outMin) pid->outputSum = pid->outMin;
    }
}

/* SetMode(...)****************************************************************
 * Allows the controller Mode to be set to manual (0) or Automatic (non-zero)
 * when the transition from manual to auto occurs, the controller is
 * automatically initialized
 ******************************************************************************/
void PID_SetMode(PID_TypeDef *pid, uint8_t Mode)
{
	if (pid == NULL) return;
    bool newAuto = (Mode == PID_AUTOMATIC);
    if(newAuto && !pid->inAuto)
    {
        /* Switch from manual to automatic mode */
        PID_Initialize(pid);
    }
    pid->inAuto = newAuto;
}

/* SetControllerDirection(...)*************************************************
 * The PID will either be connected to a DIRECT acting process (+Output leads
 * to +Input) or a REVERSE acting process(+Output leads to -Input.)  we need to
 * know which one, because otherwise we may increase the output when we should
 * be decreasing.  This is called from the constructor.
 ******************************************************************************/
void PID_SetControllerDirection(PID_TypeDef *pid, uint8_t Direction)
{
	if (pid == NULL) return;
    if(pid->inAuto && Direction != pid->controllerDirection)
    {
        pid->kp = (0.0f - pid->kp);
        pid->ki = (0.0f - pid->ki);
        pid->kd = (0.0f - pid->kd);
    }
    pid->controllerDirection = Direction;
}

/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
float PID_GetKp(PID_TypeDef *pid) { return pid->dispKp; }
float PID_GetKi(PID_TypeDef *pid) { return pid->dispKi; }
float PID_GetKd(PID_TypeDef *pid) { return pid->dispKd; }
uint8_t PID_GetMode(PID_TypeDef *pid) { return pid->inAuto ? PID_AUTOMATIC : PID_MANUAL; }
uint8_t PID_GetDirection(PID_TypeDef *pid) { return pid->controllerDirection; }
