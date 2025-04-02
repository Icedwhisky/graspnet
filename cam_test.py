import numpy as np
import cv2
import pyrealsense2 as rs

def main():
    # 配置管道
    pipeline = rs.pipeline()
    config = rs.config()
    
    # 启用彩色流和深度流
    config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

    try:
        # 启动管道
        pipeline.start(config)
    except Exception as e:
        print(f"相机连接异常：{e}")
        return

    try:
        while True:
            # 等待帧
            frames = pipeline.wait_for_frames()
            
            # 获取彩色帧和深度帧
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                continue

            # 将帧转换为numpy数组
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # 将深度图像转换为彩色图以便可视化
            depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

            # 显示图像
            cv2.imshow('Color', color_image)
            cv2.imshow('Depth', depth_colormap)

            # 按q键退出
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # 停止管道
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()