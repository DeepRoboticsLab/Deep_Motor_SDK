
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/epoll.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <linux/can/raw.h>
#include <sys/time.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "can_protocol.h"

enum SendRecvRet{
    //*******************************
    //SendRecvRet: SendRecv函数的返回值
    //*******************************
    //正常时返回0
    //发送长度错误返回-1  
    //接收超时错误返回-2
    //接收epoll错误返回-3
    //接收长度错误返回-4

    //*******************************
    //SendRecvRet: return value of SendRecv function
    //*******************************
    //return 0: when no error
    //return -1 send length error
    //return -2 receive timeout error
    //return -3 receive epoll error
    //return -4 receive length error
    kNoSendRecvError = 0,
    kSendLengthError = -1,
    kRecvTimeoutError = -2,
    kRecvEpollError = -3,
    kRecvLengthError = -4
};

//检查SendRecv函数返回值
//Check the return value of SendRecv function
void CheckSendRecvError(uint8_t motor_id, int code){
    switch (code)
    {
    case kNoSendRecvError:
        break;
    case kSendLengthError:
        printf("[ERROR] Motor with id %d kSendLengthError\r\n", (uint32_t)motor_id);
        break;
    case kRecvTimeoutError:
        printf("[WARN] Motor with id %d kRecvTimeoutError\r\n", (uint32_t)motor_id);
        break;
    case kRecvEpollError:
        printf("[ERROR] Motor with id %d kRecvEpollError\r\n", (uint32_t)motor_id);
        break;
    case kRecvLengthError:
        printf("[ERROR] Motor with id %d kRecvLengthError\r\n", (uint32_t)motor_id);
        break;        
    default:
        break;
    }
}

enum MotorErrorType{
    //*******************************
    //MotorErrorType: SendRecv函数的返回值
    //*******************************
    //全为0: 无错误
    //bit 0: 过压标志位
    //bit 1: 欠压标志位
    //bit 2: 过流标志位
    //bit 3: 关节过温标志位
    //bit 4: 驱动板过温标志位
    //bit 5: Can超时标志位

    //*******************************
    //MotorErrorType: the return value of SendRecv function
    //*******************************
    //all 0: no error
    //bit 0: over voltage flag
    //bit 1: under voltage flag
    //bit 2: over current flag
    //bit 3: motor over temp flag
    //bit 4: driver board over temp flag
    //bit 5: can timeout flag
    kMotorNoError = 0,
    kOverVoltage = (0x01 << 0),
    kUnderVoltage = (0x01 << 1),
    kOverCurrent = (0x01 << 2),
    kMotorOverTemp = (0x01 << 3),
    kDriverOverTemp = (0x01 << 4),
    kCanTimeout = (0x01 << 5)
};

//检查关节状态返回值
//Check motor state
void CheckMotorError(uint8_t motor_id, uint16_t code){
    if(code != kMotorNoError){
        if(code & kOverVoltage){
            printf("[ERROR] Motor with id: %d kOverVoltage\r\n", (uint32_t)motor_id);
        }
        if(code & kUnderVoltage){
            printf("[ERROR] Motor with id: %d kUnderVoltage\r\n", (uint32_t)motor_id);
        }
        if(code & kOverCurrent){
            printf("[ERROR] Motor with id: %d kOverCurrent\r\n", (uint32_t)motor_id);
        }
        if(code & kMotorOverTemp){
            printf("[ERROR] Motor with id: %d kMotorOverTemp\r\n", (uint32_t)motor_id);
        }
        if(code & kDriverOverTemp){
            printf("[ERROR] Motor with id: %d kDriverOverTemp\r\n", (uint32_t)motor_id);
        }
        if(code & kCanTimeout){
            printf("[ERROR] Motor with id: %d kCanTimeout\r\n", (uint32_t)motor_id);
        }
    }
}

//存储电机返回的数据
//Struct saving data from motor
typedef struct
{
    uint8_t motor_id_;
    uint8_t cmd_;
    float position_;
    float velocity_;
    float torque_;
    bool flag_;
    float temp_;
    uint16_t error_;
}MotorDATA;

//创建MotorDATA实例
//Create MotorDATA object
MotorDATA *MotorDATACreate(){
    MotorDATA *motor_data = (MotorDATA*)malloc(sizeof(MotorDATA));
    motor_data->error_ = kMotorNoError;
    return motor_data;
}

//销毁MotorDATA实例
//Destroy MotorData object
void MotorDATADestroy(MotorDATA *motor_data){
    free(motor_data);
}

//存储发向电机的数据
//Struct of cmd sending to motor
typedef struct
{
    uint8_t motor_id_;
    uint8_t cmd_;
    float position_;
    float velocity_;
    float torque_;
    float kp_;
    float kd_;
}MotorCMD;

//创建MotorCMD实例
//Create MotorCMD object
MotorCMD *MotorCMDCreate(){
    MotorCMD *motor_cmd = (MotorCMD*)malloc(sizeof(MotorCMD));
    return motor_cmd;
}

//往MotorCMD写入普通命令
//Write normal cmd into MotorCMD
void SetNormalCMD(MotorCMD *motor_cmd, uint8_t motor_id, uint8_t cmd){
    motor_cmd->motor_id_ = motor_id;
    motor_cmd->cmd_ = cmd;
}

//往MotorCMD写入控制命令
//Write control cmd into MotorCMD
void SetMotionCMD(MotorCMD *motor_cmd, uint8_t motor_id, uint8_t cmd, float position, float velocity, float torque, float kp, float kd){
    motor_cmd->motor_id_ = motor_id;
    motor_cmd->cmd_ = cmd;
    motor_cmd->position_ = position;
    motor_cmd->velocity_ = velocity;
    motor_cmd->torque_ = torque;
    motor_cmd->kp_ = kp;
    motor_cmd->kd_ = kd;
}

//销毁MotorCMD实例
//Destroy MotorCMD object
void MotorCMDDestroy(MotorCMD *motor_cmd){
    free(motor_cmd);
}

//将MotorCMD中的float数据转换为CAN协议中发送的uint数据
//Transform the float data in MotorCMD into uint data in can protocol
void FloatsToUints(const MotorCMD *param, uint8_t *data)
{
    uint16_t _position = FloatToUint(param->position_, POSITION_MIN, POSITION_MAX, SEND_POSITION_LENGTH);
    uint16_t _velocity = FloatToUint(param->velocity_, VELOCITY_MIN, VELOCITY_MAX, SEND_VELOCITY_LENGTH);
    uint16_t _torque = FloatToUint(param->torque_, TORQUE_MIN, TORQUE_MAX, SEND_TORQUE_LENGTH);
    uint16_t _kp = FloatToUint(param->kp_, KP_MIN, KP_MAX, SEND_KP_LENGTH);
    uint16_t _kd = FloatToUint(param->kd_, KD_MIN, KD_MAX, SEND_KD_LENGTH);
    data[0] = _position;
    data[1] = _position >> 8;
    data[2] = _velocity;
    data[3] = ((_velocity >> 8) & 0x3f)| ((_kp & 0x03) << 6);
    data[4] = _kp >> 2;
    data[5] = _kd;
    data[6] = _torque;
    data[7] = _torque >> 8;
}

//将CAN协议中收到的uint数据转换为MotorDATA中的float数据
//Transform the uint data in can protocol into float data in MotorDATA
void UintsToFloats(const struct can_frame *frame, MotorDATA *data)
{
    const ReceivedMotionData *pcan_data = (const ReceivedMotionData*)frame->data;
    data->position_ = UintToFloat(pcan_data->position, POSITION_MIN, POSITION_MAX, RECEIVE_POSITION_LENGTH);
    data->velocity_ = UintToFloat(pcan_data->velocity, VELOCITY_MIN, VELOCITY_MAX, RECEIVE_VELOCITY_LENGTH);
    data->torque_ = UintToFloat(pcan_data->torque, TORQUE_MIN, TORQUE_MAX, RECEIVE_TORQUE_LENGTH);
    data->flag_ = (bool)pcan_data->temp_flag;
    if(data->flag_ == kMotorTempFlag){
        data->temp_ = UintToFloat(pcan_data->temperature, MOTOR_TEMP_MIN, MOTOR_TEMP_MAX, RECEIVE_TEMP_LENGTH);
    }
    else{
        data->temp_ = UintToFloat(pcan_data->temperature, DRIVER_TEMP_MIN, DRIVER_TEMP_MAX, RECEIVE_TEMP_LENGTH);
    }
}

//结合motor_id和cmd形成CAN协议中的id
//Form the can id with motor_id and cmd
uint16_t FormCanId(uint8_t cmd, uint8_t motor_id){
    return (cmd << CAN_ID_SHIFT_BITS) | motor_id;
}

//根据MotorCMD进行所发送can帧的填充
//Fill in can frame with MotorCMD
void MakeSendFrame(const MotorCMD *cmd, struct can_frame *frame_ret){
    frame_ret->can_id = FormCanId(cmd->cmd_, cmd->motor_id_);
    switch (cmd->cmd_)
    {
    case ENABLE_MOTOR:
        frame_ret->can_dlc = SEND_DLC_ENABLE_MOTOR;
        break;

    case DISABLE_MOTOR:
        frame_ret->can_dlc = SEND_DLC_DISABLE_MOTOR;
        break;

    case SET_HOME:
        frame_ret->can_dlc = SEND_DLC_SET_HOME;
        break;

    case ERROR_RESET:
        frame_ret->can_dlc = SEND_DLC_ERROR_RESET;
        break;

    case CONTROL_MOTOR:
        frame_ret->can_dlc = SEND_DLC_CONTROL_MOTOR;
        FloatsToUints(cmd, frame_ret->data);
        break;

    case GET_STATUS_WORD:
        frame_ret->can_dlc = SEND_DLC_GET_STATUS_WORD;
        break;

    default:
        break;
    }
}

//根据收到的can帧进行MotorDATA的填充
//Fill in MotorDATA with can frame received
void ParseRecvFrame(const struct can_frame *frame_ret, MotorDATA *data){
    uint32_t frame_id = frame_ret->can_id;
    uint32_t cmd = (frame_id >> CAN_ID_SHIFT_BITS) & 0x3f;
    uint32_t motor_id = frame_id & 0x0f;
    data->motor_id_ = motor_id;
    data->cmd_ = cmd;
    switch (cmd)
    {
    case ENABLE_MOTOR:
        printf("[INFO] Motor with id: %d enable success\r\n", (uint32_t)motor_id);
        break;

    case DISABLE_MOTOR:
        printf("[INFO] Motor with id: %d disable success\r\n", (uint32_t)motor_id);
        break;

    case SET_HOME:
        printf("[INFO] Motor with id: %d set zero point success\r\n", (uint32_t)motor_id);
        break;

    case ERROR_RESET:
        printf("[INFO] Motor with id: %d clear error success\r\n", (uint32_t)motor_id);
        break;

    case CONTROL_MOTOR:
        UintsToFloats(frame_ret, data);
        break;    

    case GET_STATUS_WORD:
        data->error_ = (frame_ret->data[0] << 8) | frame_ret->data[1];
        break;

    default:
        printf("[WARN] Received a frame not fitting into any cmd\r\n");
        break;
    }
}

//DrMotorCan类，用于保存can的相关配置和资源
//DrMotorCan struct, saving can configs and resources
typedef struct{
    bool is_show_log_;
    int can_socket_;
    int epoll_fd_;
    pthread_mutex_t rw_mutex;
}DrMotorCan;

//创建DrMotorCan实例
//Create DrMotorCan object
DrMotorCan* DrMotorCanCreate(const char *can_name, bool is_show_log){
    DrMotorCan* can = (DrMotorCan*)malloc(sizeof(DrMotorCan));
    if(can != NULL){
        can->is_show_log_ = is_show_log;

        if((can->can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0){
            printf("[ERROR] Socket creation failed\r\n");
            exit(-1);
        }

        int flags = fcntl(can->can_socket_, F_GETFL, 0);
        if(flags == -1){
            printf("[ERROR] Getting socket flags failed\r\n");
            exit(-1);
        }
        flags |= O_NONBLOCK;
        if(fcntl(can->can_socket_, F_SETFL, flags) == -1){
            printf("[ERROR] Setting socket to non-blocking mode failed\r\n");
            exit(-1);
        }

        struct ifreq ifr;
        struct sockaddr_can addr;
        strcpy(ifr.ifr_name, can_name);
        ioctl(can->can_socket_, SIOCGIFINDEX, &ifr);
        addr.can_ifindex = ifr.ifr_ifindex;
        addr.can_family = AF_CAN;
        if(bind(can->can_socket_, (struct sockaddr*)&addr, sizeof(addr)) < 0){
            printf("[ERROR] Bind failed\r\n");
            close(can->can_socket_);
            exit(-1);
        }

        can->epoll_fd_ = epoll_create1(0);
        if(can->epoll_fd_ == -1){
            printf("[ERROR] Error creating epoll instance\r\n");
            exit(-1);
        }

        struct epoll_event event;
        event.events = EPOLLIN;
        event.data.fd = can->can_socket_;
        if(epoll_ctl(can->epoll_fd_, EPOLL_CTL_ADD, can->can_socket_, &event) == -1){
            printf("[ERROR] Adding socket to epoll failed\r\n");
            exit(-1);
        }
    }
    return can;
};

//销毁DrMotorCan实例
//Destroy DrMotorCan object
void DrMotorCanDestroy(DrMotorCan *can){
    close(can->can_socket_);
    free(can);
}

//使用DrMotorCan进行数据的发送和接收
//Send and receive data via DrMotorCan
int SendRecv(DrMotorCan *can, const MotorCMD *cmd, MotorDATA *data){
    struct can_frame send_frame, recv_frame;
    MakeSendFrame(cmd, &send_frame);

    struct timeval start_time;
    gettimeofday(&start_time, NULL);

    if(can->is_show_log_){
        printf("[INFO] Writing frame with can_id: %d, can_dlc: %d, data: %d, %d, %d, %d, %d, %d, %d, %d",
            send_frame.can_id, send_frame.can_dlc,
            (uint32_t)send_frame.data[0], (uint32_t)send_frame.data[1], (uint32_t)send_frame.data[2], (uint32_t)send_frame.data[3],
            (uint32_t)send_frame.data[4], (uint32_t)send_frame.data[5], (uint32_t)send_frame.data[6], (uint32_t)send_frame.data[7]
        );
    }
    
    pthread_mutex_lock(&can->rw_mutex);
    ssize_t nbytes1 = write(can->can_socket_, &send_frame, sizeof(send_frame));
    pthread_mutex_unlock(&can->rw_mutex);
    if(nbytes1 != sizeof(send_frame)){
        return kSendLengthError;
    }

    struct epoll_event events;
    int epoll_wait_result = epoll_wait(can->epoll_fd_, &events, 5, 3);
    if(epoll_wait_result == 0){
        return kRecvTimeoutError;
    }else if (epoll_wait_result == -1){
        return kRecvEpollError;
    }else{
        pthread_mutex_lock(&can->rw_mutex);
        ssize_t nbytes2 = read(can->can_socket_, &recv_frame, sizeof(recv_frame));
        pthread_mutex_unlock(&can->rw_mutex);
        if(nbytes2 != sizeof(recv_frame)){
            return kRecvLengthError;
        }

        if(can->is_show_log_){
            printf("[INFO] Reading frame with can_id: %d, can_dlc: %d, data: %d, %d, %d, %d, %d, %d, %d, %d\r\n",
                recv_frame.can_id, recv_frame.can_dlc,
                (uint32_t)recv_frame.data[0], (uint32_t)recv_frame.data[1], (uint32_t)recv_frame.data[2], (uint32_t)recv_frame.data[3],
                (uint32_t)recv_frame.data[4], (uint32_t)recv_frame.data[5], (uint32_t)recv_frame.data[6], (uint32_t)recv_frame.data[7]
            );
            struct timeval end_time;
            gettimeofday(&end_time, NULL);
            long long duration_us = (end_time.tv_sec - start_time.tv_sec) * 1000000LL +
                        (end_time.tv_usec - start_time.tv_usec);
            printf("[INFO] SendRecv() t_diff: %lld us\r\n", duration_us);
        }

        ParseRecvFrame(&recv_frame, data);
        return kNoSendRecvError;
    }
}
