import os
import cv2
import sys
import numpy as np
import pyrealsense2 as rs
import time
from roh_registers_v1 import *
from robotic_arm_package.robotic_arm import *
import ctypes  # Added import for ctypes

# 初始化配置
ARM_IP = "192.168.1.19"
COM_PORT = 1
NODE_ID = 2

def L_BYTE(v):
    return v & 0xFF

def H_BYTE(v):
    return (v >> 8) & 0xFF

# 新增带重试机制的写入函数
def write_with_retry(robot,port, address, data_values, max_retries=4):
    for _ in range(max_retries):
        gesture_bytes = []
        for value in data_values:
            gesture_bytes.append(H_BYTE(value))
            gesture_bytes.append(L_BYTE(value))
        
        # 转换为 ctypes 需要的字节数组格式
        data_bytes = (ctypes.c_byte * len(gesture_bytes))(*gesture_bytes)
        resp = robot.Write_Registers(port, address, len(data_values), data_bytes, NODE_ID, True)
        
        if resp == 0:  # 成功时返回0
            return True
        time.sleep(2)  # 失败时等待2秒后重试
    return False

def roh_init(robot):
    # 初始化机械臂
    robot.Close_Modbustcp_Mode()
    robot.Set_Modbus_Mode(1, 115200, 1, True)
   
    # 张开（带重试）
    success = write_with_retry(robot,COM_PORT, ROH_FINGER_POS_TARGET0, [0]*6)
    time.sleep(2)

     # 三指闭合加拇指选择90度（带重试）
    success = write_with_retry(robot,COM_PORT, ROH_FINGER_POS_TARGET2, [65535]*4)
    time.sleep(2)



# 抓取物体的函数
def grasp_object(robot):    
    # 握食指（带重试）
    success = write_with_retry(robot,COM_PORT, ROH_FINGER_POS_TARGET1, [65535])
    time.sleep(2)


def realse_object(robot):
    # 张食指（带重试）
    success = write_with_retry(robot,COM_PORT, ROH_FINGER_POS_TARGET1, [0])
    time.sleep(2)



 

if __name__ == "__main__":
    robot = Arm(RM65, ARM_IP)
    roh_init(robot)
    grasp_object(robot)
    realse_object(robot)