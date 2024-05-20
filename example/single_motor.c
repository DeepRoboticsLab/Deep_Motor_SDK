
#include "example.h"

int main(){
    signal(SIGINT, sigint_handler);
    printf("[INFO] Started single motor control\r\n");

    //创建基于socketcan的can0设备对象
    //Create an socketcan-based can0 device object     
    DrMotorCan *can = DrMotorCanCreate("can0", true);
    MotorCMD *motor_cmd = MotorCMDCreate();
    MotorDATA *motor_data = MotorDATACreate();

    //使能关节
    //Enable motor
    uint8_t motor_id = 1;
    SetNormalCMD(motor_cmd, motor_id, ENABLE_MOTOR);
    SendRecv(can, motor_cmd, motor_data);

    //创建线程，每秒检查一次关节的状态
    //Create a thread to check motor state, once per second
    pthread_t thread_id;
    MotorCheckThreadParam param;
    param.can = can;
    param.motor_id = 1;
    if(pthread_create(&thread_id, NULL, MotorStateCheckThreadFunc, (void*)&param) != 0){
        fprintf(stderr, "Failed to create thread.\n");
        return 1;
    }

    //发送控制命令
    //Send control cmd    
    while(!break_flag)
    {
        LoopControl(can, motor_cmd, motor_id, motor_data);
    }
    printf("[INFO] main thread loop stoped\r\n");

    //停止检查关节状态的线程
    //Stop the checking thread   
    if (pthread_join(thread_id, NULL) != 0) {
        fprintf(stderr, "Failed to join thread.\n");
        return 1;
    }

    //失能关节
    //Disable motor
    SetNormalCMD(motor_cmd, motor_id, DISABLE_MOTOR);
    SendRecv(can, motor_cmd, motor_data);

    //回收资源
    //Reclaim allocated memory    
    DrMotorCanDestroy(can);
    MotorCMDDestroy(motor_cmd);
    MotorDATADestroy(motor_data);

    printf("[INFO] Ended single motor control\r\n");
    return 0;
}
