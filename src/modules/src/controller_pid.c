
#include "stabilizer.h"
#include "stabilizer_types.h"

#include "attitude_controller.h"
#include "sensfusion6.h"
#include "position_controller.h"
#include "controller_pid.h"

#include "log.h"
#include "param.h"
#include "math3d.h"

#define ATTITUDE_UPDATE_DT    (float)(1.0f/ATTITUDE_RATE)

static bool tiltCompensationEnabled = false;

static attitude_t attitudeDesired;
static attitude_t rateDesired;
static float actuatorThrust;

static float cmd_thrust;
static float cmd_roll;
static float cmd_pitch;
static float cmd_yaw;
static float r_roll;
static float r_pitch;
static float r_yaw;
static float accelz;

void controllerPidInit(void)
{
  attitudeControllerInit(ATTITUDE_UPDATE_DT);           // CHECK THIS
  positionControllerInit();
}

bool controllerPidTest(void)
{
  bool pass = true;

  pass &= attitudeControllerTest();

  return pass;
}

static float capAngle(float angle) {
  float result = angle;

  while (result > 180.0f) {
    result -= 360.0f;
  }

  while (result < -180.0f) {
    result += 360.0f;
  }

  return result;
}

void controllerPid(control_t *control, setpoint_t *setpoint,                            // Input control signal (This will end up being the output) , desired state
                                         const sensorData_t *sensors,                   // Input sensor data
                                         const state_t *state,                          // Input current state
                                         const uint32_t tick)                           // Input timestamp
{
  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {                                       // Frequency Adjustment
    // Rate-controled YAW is moving YAW angle setpoint
    if (setpoint->mode.yaw == modeVelocity) {                                       // If desired yaw is in velocity mode
       attitudeDesired.yaw += setpoint->attitudeRate.yaw * ATTITUDE_UPDATE_DT;      // change attitudeDesired.yaw like this (this is an attitude struct
    } 
    else {
      attitudeDesired.yaw = setpoint->attitude.yaw;                                 // otherwise desired yaw is what is inputted in the setpoint
    }

    attitudeDesired.yaw = capAngle(attitudeDesired.yaw);                            // alter desired yaw angle one last time
    attitudeControllerDirectPID(&actuatorThrust, &rateDesired, setpoint, state);                  // Run new controller to get control signal
    attitudeControllerGetActuatorOutput(&control->roll, &control->pitch, &control->yaw);




  /*
  if (RATE_DO_EXECUTE(POSITION_RATE, tick)) {                                       // This means run at attitude controller rate of 100 Hz
    positionController(&actuatorThrust, &attitudeDesired, setpoint, state);         // Run position controller to change the attitude desired value
  }

  if (RATE_DO_EXECUTE(ATTITUDE_RATE, tick)) {                                   // This means run at attitude controller rate of 500 Hz
    // Switch between manual and automatic position control
    if (setpoint->mode.z == modeDisable) {                                          // if z position mode is disabled
      actuatorThrust = setpoint->thrust;                                            // thrust is the setpoint thrust
    }
    if (setpoint->mode.x == modeDisable || setpoint->mode.y == modeDisable) {       // if y or x position are disabled
      attitudeDesired.roll = setpoint->attitude.roll;                               // roll is the setpoint roll
      attitudeDesired.pitch = setpoint->attitude.pitch;                             // pitch is the setpoint pitch
    }
                                                                                    // at this point attituteDesired should be filled
                                                                                    // Actuator thrust should also be defined

    attitudeControllerCorrectAttitudePID(state->attitude.roll, state->attitude.pitch, state->attitude.yaw,
                                attitudeDesired.roll, attitudeDesired.pitch, attitudeDesired.yaw,
                                &rateDesired.roll, &rateDesired.pitch, &rateDesired.yaw);               // Use state and attitude desired to change rate desired, turns units from deg to deg/s
                                                                                                        // PID controller is used for this part
    // For roll and pitch, if velocity mode, overwrite rateDesired with the setpoint
    // value. Also reset the PID to avoid error buildup, which can lead to unstable
    // behavior if level mode is engaged later
    if (setpoint->mode.roll == modeVelocity) {                                      // if mode of setpoint has velocity mode for roll
      rateDesired.roll = setpoint->attitudeRate.roll;                               // overwrite roll for rateDesired with attitudeRate.roll from setpoint
      attitudeControllerResetRollAttitudePID();                                     // Reset PID ********
    }
    if (setpoint->mode.pitch == modeVelocity) {                                     // Do same for pitch
      rateDesired.pitch = setpoint->attitudeRate.pitch;
      attitudeControllerResetPitchAttitudePID();
    }

    // TODO: Investigate possibility to subtract gyro drift.
    attitudeControllerCorrectRatePID(sensors->gyro.x, -sensors->gyro.y, sensors->gyro.z,    // Take pid updated roll pitch ywa rates from last function and insert it into this one
                             rateDesired.roll, rateDesired.pitch, rateDesired.yaw);         // Takes direct error from gyroscope measurements which are in deg/s and puts it through another PID
                                                                                            // Updates them to rollOutput, pitchoutput, yawoutput
    attitudeControllerGetActuatorOutput(&control->roll,                                 // BINGO this takes the control signal and changes the roll pitch and yaw to rollOutput, PitchOutput and YawOutput
                                        &control->pitch,                                // Now Control Signal units are torques
                                        &control->yaw);

    
    
    */


    control->yaw = -control->yaw;                                                       // flips yaw torque

    cmd_thrust = control->thrust;                                                       // This is all for logging purposes
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;
    r_roll = radians(sensors->gyro.x);
    r_pitch = -radians(sensors->gyro.y);
    r_yaw = radians(sensors->gyro.z);
    accelz = sensors->acc.z;
  }
  

  if (tiltCompensationEnabled)
  {
    control->thrust = actuatorThrust / sensfusion6GetInvThrustCompensationForTilt();    // Finally control signal's thrust is changed to the desired value 
  }
  else
  {
    control->thrust = actuatorThrust;
  }

  if (control->thrust == 0)
  {
    control->thrust = 0;
    control->roll = 0;
    control->pitch = 0;
    control->yaw = 0;

    cmd_thrust = control->thrust;
    cmd_roll = control->roll;
    cmd_pitch = control->pitch;
    cmd_yaw = control->yaw;

    attitudeControllerResetAllPID();
    positionControllerResetAllPID();

    // Reset the calculated YAW angle for rate control
    attitudeDesired.yaw = state->attitude.yaw;
  }
}

/**
 * Logging variables for the command and reference signals for the
 * altitude PID controller
 */
LOG_GROUP_START(controller)
/**
 * @brief Thrust command
 */
LOG_ADD(LOG_FLOAT, cmd_thrust, &cmd_thrust)
/**
 * @brief Roll command
 */
LOG_ADD(LOG_FLOAT, cmd_roll, &cmd_roll)
/**
 * @brief Pitch command
 */
LOG_ADD(LOG_FLOAT, cmd_pitch, &cmd_pitch)
/**
 * @brief yaw command
 */
LOG_ADD(LOG_FLOAT, cmd_yaw, &cmd_yaw)
/**
 * @brief Gyro roll measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_roll, &r_roll)
/**
 * @brief Gyro pitch measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_pitch, &r_pitch)
/**
 * @brief Yaw  measurement in radians
 */
LOG_ADD(LOG_FLOAT, r_yaw, &r_yaw)
/**
 * @brief Acceleration in the zaxis in G-force
 */
LOG_ADD(LOG_FLOAT, accelz, &accelz)
/**
 * @brief Thrust command without (tilt)compensation
 */
LOG_ADD(LOG_FLOAT, actuatorThrust, &actuatorThrust)
/**
 * @brief Desired roll setpoint
 */
LOG_ADD(LOG_FLOAT, roll,      &attitudeDesired.roll)
/**
 * @brief Desired pitch setpoint
 */
LOG_ADD(LOG_FLOAT, pitch,     &attitudeDesired.pitch)
/**
 * @brief Desired yaw setpoint
 */
LOG_ADD(LOG_FLOAT, yaw,       &attitudeDesired.yaw)
/**
 * @brief Desired roll rate setpoint
 */
LOG_ADD(LOG_FLOAT, rollRate,  &rateDesired.roll)
/**
 * @brief Desired pitch rate setpoint
 */
LOG_ADD(LOG_FLOAT, pitchRate, &rateDesired.pitch)
/**
 * @brief Desired yaw rate setpoint
 */
LOG_ADD(LOG_FLOAT, yawRate,   &rateDesired.yaw)
LOG_GROUP_STOP(controller)


/**
 * Controller parameters
 */
PARAM_GROUP_START(controller)
/**
 * @brief Nonzero for tilt compensation enabled (default: 0)
 */
PARAM_ADD(PARAM_UINT8, tiltComp, &tiltCompensationEnabled)
PARAM_GROUP_STOP(controller)
