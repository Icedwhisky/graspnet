import yaml
from libs.auxiliary import get_ip, popup_message
from robotic_arm_package.robotic_arm import Arm
import sys

def get_current_pose():
    robot_ip = get_ip()
    if not robot_ip:
        popup_message("错误", "机械臂IP未连接")
        sys.exit(1)
    
    with open("config.yaml") as f:
        ROBOT_TYPE = yaml.safe_load(f).get("ROBOT_TYPE")
    print(ROBOT_TYPE)
    robot = Arm(ROBOT_TYPE, robot_ip)
    error_code, joints, current_pose, *_ = robot.Get_Current_Arm_State()
    
    if error_code == 0:
        print("当前机械臂位姿（单位：毫米/度）:")
        print(f"X: {current_pose[0]:.2f}")
        print(f"Y: {current_pose[1]:.2f}")
        print(f"Z: {current_pose[2]:.2f}")
        print(f"Rx: {current_pose[3]:.2f}")
        print(f"Ry: {current_pose[4]:.2f}")
        print(f"Rz: {current_pose[5]:.2f}")
    else:
        print(f"获取状态失败，错误码：{error_code}")

if __name__ == "__main__":
    get_current_pose()
