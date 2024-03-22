
#pragma once

#include <signal.h>
#include <unistd.h>

#include "../sdk/deep_motor_sdk.h"

typedef struct{
    DrMotorCan *can;
    uint8_t motor_id;
}MotorCheckThreadParam;

volatile sig_atomic_t break_flag = 0;
void sigint_handler(int sig) {
    break_flag = 1;
}

//每秒发送一次命令，检查can总线上关节的状态
void *MotorStateCheckThreadFunc(void *args){
    MotorCheckThreadParam *params = (MotorCheckThreadParam *)args;
    MotorCMD *motor_cmd = MotorCMDCreate();
    MotorDATA *motor_data = MotorDATACreate();
    SetNormalCMD(motor_cmd, params->motor_id, GET_STATUS_WORD);
    while(!break_flag){
        int ret = SendRecv(params->can, motor_cmd, motor_data);
        CheckSendRecvError(params->motor_id, ret);
        CheckMotorError(params->motor_id, motor_data->error_);
        sleep(1);
    }
    MotorCMDDestroy(motor_cmd);
    MotorDATADestroy(motor_data);
    printf("[INFO] motor_state_check_thread for motor with id: %d stoped\r\n", (uint32_t)params->motor_id);
}

void LoopControl(DrMotorCan *can, MotorCMD *motor_cmd, int motor_id, MotorDATA *motor_data){
    int ret = 0;
    SetMotionCMD(motor_cmd, motor_id, CONTROL_MOTOR,0,0,0.5,0,0);
    ret = SendRecv(can, motor_cmd, motor_data);
    CheckSendRecvError(motor_id, ret);
    usleep(1000);
};
