'''
云深处J60-10电机
Can 通讯程序
'''
import enum
import numba
import numpy as np
from numba import njit
import struct


# @njit(cache=True)
def float_to_uint(x, x_min, x_max, bits):
    """
    Converts a float to an unsigned int, given range and number of bits.
    """
    span = x_max - x_min
    offset = x_min
    # Python automatically handles the conversion of float to int
    return int((x - offset) * ((1 << bits) - 1) / span)

# @njit(cache=True)
def uint_to_float(x_int, x_min, x_max, bits):
    """
    Converts unsigned int to float, given range and number of bits.
    """
    span = x_max - x_min
    offset = x_min
    return (x_int * span / ((1 << bits) - 1)) + offset

# @njit(cache=True)
def FloatsToUints(data_send_orgin):
    position, velocity, torque, kp, kd = (data_send_orgin[0], data_send_orgin[1],
                                          data_send_orgin[2], data_send_orgin[3],
                                          data_send_orgin[4])
    position = float_to_uint(position, x_min=-40, x_max=40, bits=16)
    velocity = float_to_uint(velocity, x_min=-40, x_max=40, bits=14)
    torque = float_to_uint(torque, x_min=-40, x_max=40, bits=16)
    kp = float_to_uint(kp, x_min=0, x_max=1023, bits=10)
    kd = float_to_uint(kd, x_min=0, x_max=51, bits=8)
    # print(position, velocity, torque, kp, kd)
    data = [
        position & 0xff,
        position >> 8,
        velocity & 0xff,
        ((velocity >> 8) & 0x3f) | ((kp & 0x03) << 6),
        kp >> 2,
        kd,
        torque & 0xff,
        torque >> 8
    ]
    return data


# @njit(cache=True)
def UintsToFloats(data_recv_orgin):
    # data = [212, 253, 153, 249, 127, 178, 128, 62]
    #print(f"data is {data_recv_orgin}")
    if len(data_recv_orgin) == 8:
        data = data_recv_orgin

        position = (data[0])|(data[1] << 8)|((data[2]&0x0F)<<16)
        velocity = ((data[2]&0xF0))|(data[3] << 4)|(data[4]<<12)
        torque = (data[5])|(data[6]<<8)
        temp = (data[7] & 0xFF)>>1
        R_flag = np.bool_(data[7] & 0x01)

        R_position = uint_to_float(position, x_min=-40, x_max=40, bits=20)
        R_velocity = uint_to_float(velocity, x_min=-40, x_max=40, bits=20)
        R_torque = uint_to_float(torque, x_min=-40, x_max=40, bits=16)

        if R_flag == 0:
            temp = uint_to_float(temp, x_min=-20, x_max=200, bits=7)
        elif R_flag == 1:
            temp = uint_to_float(temp, x_min=-20, x_max=200, bits=7)

        data_recv = [R_position, R_velocity, R_torque, R_flag, temp]
    else:
        data_recv = [0, 0, 0, 0, 0]

    return data_recv

def FormSendData(cmd, id, data_send_orgin):
    can_id = (cmd << 5) | id
    data = [0] * 8
    if cmd == 1:
        can_dlc = 0  # 断使能
    elif cmd == 2:
        can_dlc = 0  # 使能
    elif cmd == 4:
        can_dlc = 8  # 控制电机
        data = FloatsToUints(data_send_orgin)
    data_send_can = [
        can_id, can_dlc, data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]
    ]
    return data_send_can

def ParseRecvData(id, data_recv_orgin):
    cmd = (id >> 5) & 0x3f
    id = id & 0x0F
    data_recv = [0] * 5
    #print(f"cmd is {cmd}")
    if cmd == 1:
        print(f"[INFO] Motor with id: {id} disable success\r\n")
    elif cmd == 2:
        print(f"[INFO] Motor with id: {id} enable success\r\n")
    elif cmd == 4:
        data_recv = UintsToFloats(data_recv_orgin)
    data_recv_can = [
        id, cmd, data_recv[0], data_recv[1], data_recv[2], data_recv[3], data_recv[4]
    ]
    return data_recv_can


data_recv_can = ParseRecvData(id=144, data_recv_orgin=[140, 36, 120, 135, 129, 243, 127, 66])
#print(data_recv_can)
