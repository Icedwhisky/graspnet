import yaml
from libs.auxiliary import create_folder_with_date, get_ip, popup_message
import sys
import cv2
import numpy as np
import pyrealsense2 as rs
from scipy.spatial.transform import Rotation as R
from robotic_arm_package.robotic_arm import *
from vertical_grab.convert_update import convert_new
from cv_process import segment_image
from grasp_process import run_grasp_inference
from roh_hand import roh_init,grasp_object,realse_object

# 相机内参  640*480
color_intr = {"ppx": 327.675, "ppy": 245.181, "fx": 608.403, "fy": 608.064}
depth_intr = {"ppx": 323.422, "ppy": 241.969, "fx": 388.001, "fy": 388.001}

# 相机内参  1280*720
color_intr = {"ppx": 651.513, "ppy": 367.771, "fx": 912.605, "fy": 912.096}
depth_intr = {"ppx": 645.704, "ppy": 363.281, "fx": 646.669, "fy": 646.669}

# 手眼标定外参 新：20250401
rotation_matrix = [
    [0.00075793, 0.99984082, 0.01782573],  
    [-0.99990618, 0.00051393, 0.01368855], 
    [0.01367721, -0.01783443, 0.9997474]    
]
translation_vector = [-0.13374957, 0.05191696, -0.08151545]


# 全局变量
global color_img, depth_img, robot, first_run

# 初始 关节 
init = [-29 , 0.2 , 71 , 4.7, 97, 0.2 ]

# 放下 关节
fang = [-92 , 7 , 77 , 2.6, 95, 0.2 ]

color_img = None
depth_img = None
robot = None
first_run = True  # 新增首次运行标志

def get_aligned_frame(self):
        align = rs.align(rs.stream.color)  # type: ignore
        frames = self.pipline.wait_for_frames()
        # aligned_frames 对齐之后结果
        aligned_frames = align.process(frames)
        color = aligned_frames.get_color_frame()
        depth = aligned_frames.get_depth_frame()
        return color, depth

def callback(color_frame, depth_frame):
    global color_img, depth_img
    scaling_factor_x = 1
    scaling_factor_y = 1

    color_img = cv2.resize(
        color_frame, None,
        fx=scaling_factor_x,
        fy=scaling_factor_y,
        interpolation=cv2.INTER_AREA
    )
    depth_img = cv2.resize(
        depth_frame, None,
        fx=scaling_factor_x,
        fy=scaling_factor_y,
        interpolation=cv2.INTER_NEAREST
    )

    if color_img is not None and depth_img is not None:
         test_grasp()

def pose_to_list(pose):
    # 从 Pose 对象中提取位置和欧拉角信息
    x = pose.position.x
    y = pose.position.y
    z = pose.position.z
    rx = pose.euler.rx
    ry = pose.euler.ry
    rz = pose.euler.rz
    return [x, y, z, rx, ry, rz]

def matrix_to_list(T1):
    # 从 Pose 对象中提取位置和欧拉角信息
    T_ee2base =  T1.data
    return T_ee2base

def numpy_to_Matrix(nparr):
    # 假设 nparr 是一个 4x4 的 numpy 数组
    mat = Matrix()
    mat.irow = 4
    mat.iline = 4
    # 填充 data 字段
    for i in range(4):
        for j in range(4):
            mat.data[i][j] = float(nparr[i, j])
    return mat


def test_grasp():
    global color_img, depth_img, robot, first_run

    if color_img is None or depth_img is None:
        print("[WARNING] Waiting for image data...")
        return

    # 图像处理部分
    masks = segment_image(color_img)  
    
    translation, rotation_mat_3x3, width = run_grasp_inference(
        color_img,
        depth_img,
        masks
    )

    # 首次运行只计算不执行
    if first_run:
        print("[INFO] 首次运行模拟完成，准备正式执行")
        first_run = False
        return  # 直接返回不执行后续动作

    print(f"[DEBUG] Grasp预测结果 - 平移: {translation}, 旋转矩阵:\n{rotation_mat_3x3}")

    error_code, joints, current_pose_old, arm_err_ptr, sys_err_ptr = robot.Get_Current_Arm_State()
    print("\n[DEBUG]当前关节角度:", joints)
    print("\n[DEBUG]未补偿夹爪前位姿:", current_pose_old)

    # current_pose_first = robot.Algo_Cartesian_Tool(joints,0,0,-0.055)
    current_pose_first = robot.Algo_Cartesian_Tool(joints,0,0,-0.04)

    current_pose = pose_to_list(current_pose_first)
    print("[DEBUG] 补偿夹爪后的位姿:", current_pose)
    T1 = robot.Algo_Pos2Matrix(current_pose)  # 位姿转换为齐次矩阵
    T_ee2base = matrix_to_list(T1)
    # print("[DEBUG] 官方api计算出对应的齐次矩阵:", T_ee2base)


    T_grasp2base = convert_new(
        translation,
        rotation_mat_3x3,
        current_pose,
        rotation_matrix,
        translation_vector,
        T_ee2base
    )
    print("[DEBUG] 基坐标系抓取齐次矩阵:", T_grasp2base)

    matrix_struct = numpy_to_Matrix(T_grasp2base)
    base_pose_first = robot.Algo_Matrix2Pos(matrix_struct)
    # print("[DEBUG] base_pose_first是什么:", base_pose_first)

    base_pose = pose_to_list(base_pose_first)
    print("[DEBUG] 最终抓取位姿是什么:", base_pose)



    # 正式执行部分
    base_pose_np = np.array(base_pose, dtype=float)
    base_xyz = base_pose_np[:3]
    base_rxyz = base_pose_np[3:]


    # # 预抓取计算
    # pre_grasp_offset = 0.15
    # pre_grasp_pose = np.array(base_pose, dtype=float).copy()
    # rotation_mat = R.from_euler('xyz', pre_grasp_pose[3:]).as_matrix()
    # z_axis = rotation_mat[:, 2]
    # pre_grasp_pose[:3] -= z_axis * pre_grasp_offset


    # 运动控制
    grasp_pose = np.concatenate([base_xyz, base_rxyz]).tolist()
    print(f"[DEBUG] 调整后的抓取位姿: {grasp_pose}")



    try:
        # print(f"预抓取位姿: {pre_grasp_pose.tolist()}")
        # ret = robot.Movej_P_Cmd(pre_grasp_pose.tolist(), 5)
        # if ret != 0: raise RuntimeError(f"预抓取失败，错误码: {ret}")
        

        print(f"实际抓取: {base_pose}")
        ret = robot.Movej_P_Cmd(base_pose, 5)
        if ret != 0: raise RuntimeError(f"抓取失败，错误码: {ret}")

        # print("闭合夹爪")
        # ret = robot.Set_Gripper_Pick(200, 300)
        # if ret != 0: raise RuntimeError(f"夹爪闭合失败，错误码: {ret}")

        print("闭合夹爪")
        grasp_object(robot)

        robot.Movej_Cmd(init, 10, 0)
        robot.Movej_Cmd(fang, 10, 0)
        realse_object(robot)
        # robot.Set_Gripper_Release(200)
        robot.Movej_Cmd(init, 10, 0)
    except Exception as e:
        print(f"[ERROR] 运动异常: {str(e)}")
        robot.Movej_Cmd(init, 10, 0)



def displayD435():
    global first_run
    # 配置管道
    pipeline = rs.pipeline()
    config = rs.config()
    
    # 启用彩色流和深度流
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
    print("pipeline_config")
    
    try:
        profile = pipeline.start(config)
        color_sensor = profile.get_device().query_sensors()[1]
        color_sensor.set_option(rs.option.enable_auto_exposure, 1)
        # print("pipeline_start")
        # 新增：创建对齐对象，将深度图与彩色图对齐
        align = rs.align(rs.stream.color)  # 对齐到彩色图像流

        while True:
            frames = pipeline.wait_for_frames()
            if not frames:
                continue

            # 对齐帧
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if not color_frame or not depth_frame:
                continue

            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())

            callback(color_image, depth_image)

            # print("callback work")

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break

    finally:
        pipeline.stop()
        cv2.destroyAllWindows()


def main():
    global robot, first_run
    robot_ip = get_ip()
    logger_.info(f'robot_ip:{robot_ip}')

    if robot_ip:
        with open("config.yaml", 'r', encoding='utf-8') as file:
            data = yaml.safe_load(file)
        ROBOT_TYPE = data.get("ROBOT_TYPE")
        robot = Arm(ROBOT_TYPE, robot_ip)
        robot.Change_Work_Frame()
        print(robot.API_Version())
        roh_init(robot)
    else:
        popup_message("提醒", "机械臂 IP 没有 ping 通")
        sys.exit(1)

    # 初始化设置  
    robot.Movej_Cmd(init, 10, 0)

    # 重置首次运行标志
    first_run = True
    displayD435()


if __name__ == "__main__":
    def get_aligned_frame(self):
        align = rs.align(rs.stream.color)  # type: ignore
        frames = self.pipline.wait_for_frames()
        # aligned_frames 对齐之后结果
        aligned_frames = align.process(frames)
        color = aligned_frames.get_color_frame()
        depth = aligned_frames.get_depth_frame()
        return color, depth
    main()


    