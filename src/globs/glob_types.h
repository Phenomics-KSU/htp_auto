// Define types of all globs. Instances are defined in globs.cpp
// To use globs include globs.h.
// To define new objects you must modify this file and globs.h

#ifndef GLOB_TYPES_INCLUDED_H
#define GLOB_TYPES_INCLUDED_H

// Includes
#include <stdint.h>
#include "telemetry_text_size.h"

/****************************************************************************************/
// commands to the guidance. 
typedef struct 
{
     float speed;          // [m/s] linear speed
     float omega;          // [rad/s] angular rate
} glo_motion_commands_t;

/****************************************************************************************/
// uncalibrated accel data. 
typedef struct 
{
     float accels[3];       // [m/sec/sec]
} glo_raw_accels_t;

/****************************************************************************************/
// raw voltages from adc. 
typedef struct 
{
    float voltages[9];
} glo_raw_analog_t;

/****************************************************************************************/
// gyro and accel data.  
typedef struct 
{
     float gyros[3];        // [rad/sec]
     float accels[3];       // [m/sec/sec]
} glo_gyros_accels_t;

/****************************************************************************************/
// roll pitch yaw estimates.  
typedef struct 
{
     float rpy[3];           // [radians]
} glo_roll_pitch_yaw_t;

/****************************************************************************************/
// quaternion representation of orientation.  
typedef struct 
{
     float q[4];                // [dimensionless]
} glo_quaternion_t;

/****************************************************************************************/
// zero angle when balanced.  
typedef struct 
{
    float theta;       // [rad]
} glo_theta_zero_t;

/****************************************************************************************/
// things calculated from odometry.  
typedef struct 
{
    float left_distance;    // [m] distance left wheel has traveled   
    float right_distance;   // [m] distance right wheel has traveled
    float avg_distance;     // [m] average of left and right wheel distances
    float yaw;              // [rad] heading angle as found from odometry
    float left_speed;       // [m/s] left wheel speed
    float right_speed;      // [m/s] right whell speed
    float avg_speed;        // [m/s] average of left and right wheel speeds
} glo_odometry_t;

/****************************************************************************************/
// pid type used for several objects.  
typedef struct 
{
    float Kp;
    float Ki;
    float Kd;
    float integral_lolimit;
    float integral_hilimit;
    float lolimit;
    float hilimit;
} glo_pid_params_t;

/****************************************************************************************/
// operation state of the gimbal. NOT ACTUALLY A GLOB. Used in operation_modes.
typedef enum 
{
    normal,
    waiting_for_push_button,
    getting_theta_zero,
    debugging,
} glo_op_state_t;

/****************************************************************************************/
// sets operation modes. 
typedef struct 
{
    glo_op_state_t op_state;
} glo_modes_t;

/****************************************************************************************/
// Outgoing telemetry assert message for user feedback. Automatically sent when assert fails.
typedef struct 
{
    uint32_t action; // corresponds to enum in util_assert.h (ie continue, restart, stop, etc)
    char text[telemetry_text_size];
    
} glo_assert_message_t;

/****************************************************************************************/
// Outgoing telemetry debug log/status message.  Separate message to prevent overwriting 
// assert message in telemetry queue. Use debug_printf() to send message.
typedef struct 
{
    char text[telemetry_text_size];
    
} glo_debug_message_t;

/****************************************************************************************/
// Data that is transmitted when a capture command is received.
typedef struct 
{
    float time; // seconds 
    float d1;
    float d2;
    float d3;
    float d4;
    float d5;
    float d6;
    float d7;
    float d8;
    
} glo_capture_data_t;

/****************************************************************************************/
// List of supported driving commands IDs. Not actually a glob.
typedef uint32_t driving_command_id_t;
enum
{
    DRIVING_COMMAND_FORWARD,
    DRIVING_COMMAND_REVERSE,
    DRIVING_COMMAND_RIGHT,
    DRIVING_COMMAND_LEFT,
    DRIVING_COMMAND_STOP,
    DRIVING_COMMAND_SAR,
};

/****************************************************************************************/
// Command that is received by user for driving robot.
typedef struct 
{
    driving_command_id_t movement_type;
    float speed;
    float omega;
    
} glo_driving_command_t;

/****************************************************************************************/
// Command that is received by user for starting/stopping data logging (glo_capture_data_t)
typedef struct 
{
    uint8_t is_start; // false (0) if should stop sending data
    uint8_t pad0;
    uint16_t frequency; // Hz
    uint32_t desired_samples; // How many samples to collect before stopping.
    uint32_t total_samples; // Used to notify UI samples are done being sent and how many there should be.
    
} glo_capture_command_t;

/****************************************************************************************/
// For sending low rate periodic status data for user feedback.
typedef struct 
{
    float tilt_angle; // radians
    
} glo_status_data_t;

/****************************************************************************************/
// Motor current commands
typedef struct 
{
    float amps[2]; // [Amp] 
    
} glo_current_commands_t;


#endif // GLOB_TYPES_INCLUDED_H
