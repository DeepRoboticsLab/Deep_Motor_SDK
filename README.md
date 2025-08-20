# Deep_Motor_SDK

[简体中文](./README_ZH.md)

Developers can use this SDK to communicate with Driver via CAN bus, enable and disable motor, send control paramters and etc. Example codes for single-joint control ***single_motor*** and multi-joint control ***multi_motor*** are provided in the ***example*** folder.   
The ***tools*** folder contains ***joint debugger software*** which can be used with reference to the product manual. Windows version (.exe) supports Windows 10 and Windows 11, and Ubuntu version (.AppImage) supports Ubuntu 22 or newer.
There are two methods to install the joint debugging software on Ubuntu. You may choose either method:

- Method 1：
Open a terminal in the directory where the installation package `DeepMotorTool-v1.9.3.AppImag` is located, and execute the following command:
```bash
./DeepMotorTool-v1.9.3.AppImage --appimage-extract
cd squashfs-root
./AppRun
```

- Method 2：
Open a terminal in the directory where the installation package `DeepMotorTool-v1.9.3.AppImag` is located, and execute the following command:
```bash
sudo apt update
sudo apt install fuse libfuse2
./DeepMotorTool-v1.9.3.AppImage
```

## 1 Environment Dependencies
Currently, this SDK is only supported to compile and run on the Linux system.
## 2 Compilation and Running of Example Codes
### 2.1 Check CAN Interface
The original example codes use the CAN bus interface "can0" by default. Before running the codes, please enter `ip a` in the terminal to obtain the actual CAN interface name of the device being used.

<img src="./doc/cancheck.png"/>

If the CAN bus interface is not "can0", please modify "can0" to the actual port in the example codes.

```c
DrMotorCan *can = DrMotorCanCreate("can0", true);
```
Also, change "can0" in */scripts/set_up_can.sh* to the actual port.

```shell
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
```

### 2.2 Modify Motor ID
The default ID for each motor is 1.  
The project ***single_motor*** defaults to communicating with the joint whose motor_id=1. If the Motor ID of the joint has been changed, please modify the default motor_id to the actual Motor ID in the code.
```c
uint8_t motor_id = 1;
```
The project ***multi_motor*** defaults to communicating with joints with motor_id from 1 to 2. If it does not match the actual Motor ID, please make the corresponding modifications in the code.

### 2.3 Compilation

The executable files for ***single_motor*** and ***multi_motor*** are provided in the ***example*** folder. If you have made changes to the original codes, you need to recompile it by entering the command `./script/compile.sh` in a terminal in the */Deep_Motor_SDK* directory.

### 2.4 Running
#### 2.4.1 Set up and Start CAN Device
Open a terminal in the */Deep_Motor_SDK* directory and enter the command `./script/set_up_can.sh` to set the CAN baud rate and start the CAN device.
#### 2.4.2 Run single_motor
Open a terminal in the */Deep_Motor_SDK* directory and enter the command `./single_motor` to run the executable program ***single_motor***. The corresponding joint will keep rotating until the terminal is closed with `ctrl+c`.
#### 2.4.3 Run multi_motor
Open a terminal in the */Deep_Motor_SDK* directory and enter the command `./multi_motor` to run the executable program ***multi_motor***. All corresponding joints will keep rotating until the terminal is closed with `ctrl+c`.

## 3 How to Use the SDK
When using the SDK, you can refer to the example codes and include it into your code file.
```c
#include "../sdk/deep_motor_sdk.h"
```
Refer to ***single_motor.c***, you can implement the following functions using the SDK:

### 3.1 Create CAN Bus Device
```c
//Not display log
DrMotorCan *can = DrMotorCanCreate("can0", false);

//Display log
DrMotorCan *can = DrMotorCanCreate("can0", true);
```

### 3.2 Enable a Joint with Specific Motor ID

```c
uint8_t motor_id = 1;
SetNormalCMD(motor_cmd, motor_id, ENABLE_MOTOR);
SendRecv(can, motor_cmd, motor_data);
```

### 3.3 Create a Thread to Check the Status of a Specific Joint
```c
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

pthread_t thread_id;
MotorCheckThreadParam param;
param.can = can;
param.motor_id = 1;
if(pthread_create(&thread_id, NULL, MotorStateCheckThreadFunc, (void*)&param) != 0){
    fprintf(stderr, "Failed to create thread.\n");
    return 1;
}
```

### 3.4 Send Control Command to a Joint with Specific Motor ID
```c
SetMotionCMD(motor_cmd, motor_id, CONTROL_MOTOR,0,0,0.3,0,0);
ret = SendRecv(can, motor_cmd, motor_data);
CheckSendRecvError(motor_id, ret);
```

### 3.5 Disable a Joint with Specific Motor ID
```c
SetNormalCMD(motor_cmd, motor_id, DISABLE_MOTOR);
SendRecv(can, motor_cmd, motor_data);
```

### 3.6 Release Resources
```c
DrMotorCanDestroy(can);
MotorCMDDestroy(motor_cmd);
MotorDATADestroy(motor_data);
```

## Acknowledgements
The Python version of the J60 joint control [examples](./python_motor_examples) is provided by Dr Liu from Harbin Engineering University, which can be used to develop control joints based on the Python version examples by referring to the use of the C version of the SDK.
Thanks to Dr Liu [haikuo00zero](https://github.com/haikuo00zero) for his selfless open source and contribution!
