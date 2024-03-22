#pragma once

// Constants
#define CAN_ID_SHIFT_BITS 5

#define POSITION_MIN -40.0f
#define POSITION_MAX 40.0f
#define VELOCITY_MIN -40.0f
#define VELOCITY_MAX 40.0f
#define KP_MIN 0.0f
#define KP_MAX 1023.0f
#define KD_MIN 0.0f
#define KD_MAX 51.0f
#define TORQUE_MIN -40.0f
#define TORQUE_MAX 40.0f

#define GEAR_RATIO_MIN 0.0f
#define GEAR_RATIO_MAX 50.0f

#define MOTOR_TEMP_MIN -20.0f
#define MOTOR_TEMP_MAX 200.0f
#define DRIVER_TEMP_MIN -20.0f
#define DRIVER_TEMP_MAX 200.0f
#define CURRENT_MIN 0.0f
#define CURRENT_MAX 40.0f

#define SEND_POSITION_LENGTH 16
#define SEND_VELOCITY_LENGTH 14
#define SEND_KP_LENGTH 10
#define SEND_KD_LENGTH 8
#define SEND_TORQUE_LENGTH 16
#define SEND_GEAR_RATIO_LENGTH 16
#define SEND_LIMIT_CURRENT_LENGTH 16

#define RECEIVE_POSITION_LENGTH 20
#define RECEIVE_VELOCITY_LENGTH 20
#define RECEIVE_TORQUE_LENGTH 16
#define RECEIVE_TEMP_FLAG_LENGTH 1
#define RECEIVE_TEMP_LENGTH 7

#define ERROR_CODE_LENGTH 16
#define ERROR_VOLTAGE_LENGTH 16
#define ERROR_CURRENT_LENGTH 16
#define ERROR_MOTOR_TEMP_LENGTH 8
#define ERROR_DRIVER_TEMP_LENGTH 8



// Commands
#define DISABLE_MOTOR 1
#define ENABLE_MOTOR 2
#define CALIBRATE_START 3
#define CONTROL_MOTOR 4
#define RESET_MOTOR 5
#define SET_HOME 6
#define SET_GEAR 7
#define SET_ID 8
#define SET_CAN_TIMEOUT 9
#define SET_BANDWIDTH 10
#define SET_LIMIT_CURRENT 11
#define SET_UNDER_VOLTAGE 12
#define SET_OVER_VOLTAGE 13
#define SET_MOTOR_TEMPERATURE 14
#define SET_DRIVE_TEMPERATURE 15
#define SAVE_CONFIG 16
#define ERROR_RESET 17
#define WRITE_APP_BACK_START 18
#define WRITE_APP_BACK 19
#define CHECK_APP_BACK 20
#define DFU_START 21
#define GET_FW_VERSION 22
#define GET_STATUS_WORD 23
#define GET_CONFIG 24
#define CALIB_REPORT 31



// SendDLC
#define SEND_DLC_DISABLE_MOTOR 0
#define SEND_DLC_ENABLE_MOTOR 0
#define SEND_DLC_CALIBRATE_START 0
#define SEND_DLC_CONTROL_MOTOR 8
#define SEND_DLC_RESET_MOTOR 0
#define SEND_DLC_SET_HOME 0
#define SEND_DLC_SET_GEAR 2
#define SEND_DLC_SET_ID 1
#define SEND_DLC_SET_CAN_TIMEOUT 1
#define SEND_DLC_SET_BANDWIDTH 2
#define SEND_DLC_SET_LIMIT_CURRENT 2
#define SEND_DLC_SET_UNDER_VOLTAGE 2
#define SEND_DLC_SET_OVER_VOLTAGE 2
#define SEND_DLC_SET_MOTOR_TEMPERATURE 2
#define SEND_DLC_SET_DRIVE_TEMPERATURE 2
#define SEND_DLC_SAVE_CONFIG 0
#define SEND_DLC_ERROR_RESET 0
#define SEND_DLC_WRITE_APP_BACK_START 0
#define SEND_DLC_WRITE_APP_BACK 8
#define SEND_DLC_CHECK_APP_BACK 8
#define SEND_DLC_DFU_START 0
#define SEND_DLC_GET_FW_VERSION 0
#define SEND_DLC_GET_STATUS_WORD 0
#define SEND_DLC_GET_CONFIG 0
#define SEND_DLC_CALIB_REPORT 8



// ReceiveDLC
#define RECEIVE_DLC_DISABLE_MOTOR 1
#define RECEIVE_DLC_ENABLE_MOTOR 1
#define RECEIVE_DLC_CALIBRATE_START 1
#define RECEIVE_DLC_CONTROL_MOTOR 8
#define RECEIVE_DLC_RESET_MOTOR 1
#define RECEIVE_DLC_SET_HOME 1
#define RECEIVE_DLC_SET_GEAR 2
#define RECEIVE_DLC_SET_ID 1
#define RECEIVE_DLC_SET_CAN_TIMEOUT 1
#define RECEIVE_DLC_SET_BANDWIDTH 1
#define RECEIVE_DLC_SET_LIMIT_CURRENT 1
#define RECEIVE_DLC_SET_UNDER_VOLTAGE 1
#define RECEIVE_DLC_SET_OVER_VOLTAGE 1
#define RECEIVE_DLC_SET_MOTOR_TEMPERATURE 1
#define RECEIVE_DLC_SET_DRIVE_TEMPERATURE 1
#define RECEIVE_DLC_SAVE_CONFIG 1
#define RECEIVE_DLC_ERROR_RESET 1
#define RECEIVE_DLC_WRITE_APP_BACK_START 1
#define RECEIVE_DLC_WRITE_APP_BACK 1
#define RECEIVE_DLC_CHECK_APP_BACK 2
#define RECEIVE_DLC_DFU_START 1
#define RECEIVE_DLC_GET_FW_VERSION 2
#define RECEIVE_DLC_GET_STATUE_WORD 5
#define RECEIVE_DLC_GET_CONFIG 8
#define RECEIVE_DLC_CALIB_REPORT 8

#include <stdint.h>
;
#pragma pack(push, 1)
typedef union SendMotionData
{
    uint8_t data[8];
    struct
    {
        uint32_t position : SEND_POSITION_LENGTH;
        uint32_t velocity : SEND_VELOCITY_LENGTH;
        uint32_t kp : SEND_KP_LENGTH;
        uint32_t kd : SEND_KD_LENGTH;
        uint32_t torque : SEND_TORQUE_LENGTH;
    };
}SendMotionData;

typedef union ReceivedMotionData
{
    uint8_t data[8];
    struct
    {
        uint32_t position : RECEIVE_POSITION_LENGTH;
        uint32_t velocity : RECEIVE_VELOCITY_LENGTH;
        uint32_t torque : RECEIVE_TORQUE_LENGTH;
        uint32_t temp_flag : RECEIVE_TEMP_FLAG_LENGTH;
        uint32_t temperature : RECEIVE_TEMP_LENGTH;
    };
}ReceivedMotionData;

typedef union ReceivedErrorData
{
    uint8_t data[8];
    struct
    {
        uint16_t error_code : ERROR_CODE_LENGTH;
        uint16_t voltage : ERROR_VOLTAGE_LENGTH;
        uint16_t current : ERROR_CURRENT_LENGTH;
        uint8_t motor_temp : ERROR_MOTOR_TEMP_LENGTH;
        uint8_t driver_temp : ERROR_DRIVER_TEMP_LENGTH;
    };
}ReceivedErrorData;
#pragma pack(pop)

enum Temp_Flag{
    kDriverTempFlag=0,
    kMotorTempFlag=1
};

uint32_t FloatToUint(const float x, const float x_min, const float x_max, const uint8_t bits){
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (uint32_t)((x-offset)*((float)((1<<bits)-1))/span);
}
float UintToFloat(const int x_int, const float x_min, const float x_max, const uint8_t bits){
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

