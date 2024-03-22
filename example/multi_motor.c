
#include "example.h"

#define MOTOR_NUMBER 2

int main(){
    signal(SIGINT, sigint_handler);
    printf("[INFO] Started multi motor control\r\n");

    //创建基于socketcan的can0设备对象
    DrMotorCan *can = DrMotorCanCreate("can0", true);
    MotorCMD *motor_cmd = MotorCMDCreate();
    MotorDATA *motor_data = MotorDATACreate();

    //使能同一总线上的关节
    for(int i = 0; i < MOTOR_NUMBER; i++){
        int motor_id = i+1;
        SetNormalCMD(motor_cmd, motor_id, ENABLE_MOTOR);
        SendRecv(can, motor_cmd, motor_data);
    }

    //创建线程，每秒检查一次can总线上各关节的状态
    pthread_t thread_id[MOTOR_NUMBER];
    for(int i = 0; i < MOTOR_NUMBER; i++){
        MotorCheckThreadParam param;
        param.can = can;
        param.motor_id = i+1;
        if(pthread_create(&thread_id[i], NULL, MotorStateCheckThreadFunc, (void*)&param) != 0){
            fprintf(stderr, "Failed to create thread.\n");
            return 1;
        }
    }

    //发送控制命令
    while(!break_flag)
    {
        for(int i = 0; i < MOTOR_NUMBER; i++){
            int motor_id = i+1;
            LoopControl(can, motor_cmd, motor_id, motor_data);
        }
    }
    printf("[INFO] main thread loop stoped\r\n");

    //停止所有检查关节状态的线程
    for(int i = 0; i < MOTOR_NUMBER; i++){
        // 等待线程结束
        if (pthread_join(thread_id[i], NULL) != 0) {
            fprintf(stderr, "Failed to join thread %d.\n", i);
            return -1;
        }
    }

    //失能同一总线上的所有关节
    for(int i = 0; i < MOTOR_NUMBER; i++){
        int motor_id = i+1;
        SetNormalCMD(motor_cmd, motor_id, DISABLE_MOTOR);
        SendRecv(can, motor_cmd, motor_data);
    }

    //回收资源
    DrMotorCanDestroy(can);
    MotorCMDDestroy(motor_cmd);
    MotorDATADestroy(motor_data);

    printf("[INFO] Ended multi motor control\r\n");
    return 0;
}
