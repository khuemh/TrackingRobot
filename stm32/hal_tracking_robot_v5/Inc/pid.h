// 
// Header Guard
// 
#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

//*********************************************************************************
// Headers
//*********************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

// 
// C Binding for C++ Compilers
// 
#ifdef __cplusplus
extern "C"
{
#endif

//*********************************************************************************
// Macros and Globals
//*********************************************************************************

typedef enum
{
    DIRECT,
    REVERSE
} PIDDirection;

typedef struct
{
    float input;
    float lastInput;
    float output;
    
    // 
    // Gain constant values that the controller alters for
    // its own use
    // 
    float alteredKp;
    float alteredKi;
    float alteredKd;
    
    // 
    // The Integral Term
    // 
    float iTerm;
    
    float sampleTime;
    float outMin;
    float outMax;
    float setpoint;
    
    // 
    // The sense of direction of the controller
    // DIRECT:  A positive setpoint gives a positive output
    // REVERSE: A positive setpoint gives a negative output
    // 
    PIDDirection controllerDirection;

} PIDControl;

//*********************************************************************************
// Prototypes
//*********************************************************************************

// 
// PID Initialize
// Description:
//      Initializes a PIDControl instantiation. This should be called at least once
//      before any other PID functions are called.
// Parameters:
//      pid - The address of a PIDControl instantiation.
//      kp - Positive P gain constant value.
//      ki - Positive I gain constant value.
//      kd - Positive D gain constant value.
//      sampleTimeSeconds - Interval in seconds on which PIDCompute will be called.
//      minOutput - Constrain PID output to this minimum value.
//      maxOutput - Constrain PID output to this maximum value.
//      mode - Tells how the controller should respond if the user has taken over
//             manual control or not.
//             MANUAL:    PID controller is off. User can manually control the 
//                        output.
//             AUTOMATIC: PID controller is on. PID controller controls the output.
//      controllerDirection - The sense of direction of the controller
//                            DIRECT:  A positive setpoint gives a positive output.
//                            REVERSE: A positive setpoint gives a negative output.
// Returns:
//      Nothing.
// 
extern void PIDInit(PIDControl *pid, float kp, float ki, float kd, 
                    float sampleTimeSeconds, float minOutput, float maxOutput, 
                    PIDDirection controllerDirection);     	

// 
// PID Compute
// Description:
//      Should be called on a regular interval defined by sampleTimeSeconds.
//      Typically, PIDSetpointSet and PIDInputSet should be called immediately
//      before PIDCompute.
// Parameters:
//      pid - The address of a PIDControl instantiation.
// Returns:
//      True if in AUTOMATIC. False if in MANUAL.
//                     
extern bool PIDCompute(PIDControl *pid); 


// 
// PID Output Limits Set
// Description:
//      Sets the new output limits. The new limits are applied to the PID
//      immediately.
// Parameters:
//      pid - The address of a PIDControl instantiation.
//      min - Constrain PID output to this minimum value.
//      max - Constrain PID output to this maximum value.
// Returns:
//      Nothing.
// 
extern void PIDOutputLimitsSet(PIDControl *pid, float min, float max); 							  							  

// 
// PID Tunings Set
// Description:
//      Sets the new gain constant values.
// Parameters:
//      pid - The address of a PIDControl instantiation.
//      kp - Positive P gain constant value.
//      ki - Positive I gain constant value.
//      kd - Positive D gain constant value.
// Returns:
//      Nothing.
// 
extern void PIDTuningsSet(PIDControl *pid, float kp, float ki, float kd);         	                                         

// 
// PID Tuning Gain Constant P Set
// Description:
//      Sets the proportional gain constant value.
// Parameters:
//      pid - The address of a PIDControl instantiation.
//      kp - Positive P gain constant value.
// Returns:
//      Nothing.
// 
extern void PIDTuningKpSet(PIDControl *pid, float kp);

// 
// PID Tuning Gain Constant I Set
// Description:
//      Sets the proportional gain constant value.
// Parameters:
//      pid - The address of a PIDControl instantiation.
//      ki - Positive I gain constant value.
// Returns:
//      Nothing.
// 
extern void PIDTuningKiSet(PIDControl *pid, float ki);

// 
// PID Tuning Gain Constant D Set
// Description:
//      Sets the proportional gain constant value.
// Parameters:
//      pid - The address of a PIDControl instantiation.
//      kd - Positive D gain constant value.
// Returns:
//      Nothing.
// 
extern void PIDTuningKdSet(PIDControl *pid, float kd);

// 
// PID Controller Direction Set
// Description:
//      Sets the new controller direction.
// Parameters:
//      pid - The address of a PIDControl instantiation.
//      controllerDirection - The sense of direction of the controller
//                            DIRECT:  A positive setpoint gives a positive output
//                            REVERSE: A positive setpoint gives a negative output
// Returns:
//      Nothing.
// 
extern void PIDControllerDirectionSet(PIDControl *pid, 
                                      PIDDirection controllerDirection);	  									  									  									  

// 
// PID Sample Time Set
// Description:
//      Sets the new sampling time (in seconds).
// Parameters:
//      pid - The address of a PIDControl instantiation.
//      sampleTimeSeconds - Interval in seconds on which PIDCompute will be called.
// Returns:
//      Nothing.
// 
extern void PIDSampleTimeSet(PIDControl *pid, float sampleTimeSeconds);                                                       									  									  									   

// 
// Basic Set and Get Functions for PID Parameters
// 

// 
// PID Setpoint Set
// Description:
//      Alters the setpoint the PID controller will try to achieve.
// Parameters:
//      pid - The address of a PIDControl instantiation.
//      setpoint - The desired setpoint the PID controller will try to obtain.
// Returns:
//      Nothing.
// 
inline void PIDSetpointSet(PIDControl *pid, float setpoint) { pid->setpoint = setpoint; }

// 
// PID Input Set
// Description:
//      Should be called before calling PIDCompute so the PID controller will
//      have an updated input value to work with.
// Parameters:
//      pid - The address of a PIDControl instantiation.
//      input - The value the controller will work with.
// Returns:
//      Nothing.
// 
inline void PIDInputSet(PIDControl *pid, float input) { pid->input = input; }

// 
// End of C Binding
// 
#ifdef __cplusplus
}
#endif

#endif  // PID_CONTROLLER_H
