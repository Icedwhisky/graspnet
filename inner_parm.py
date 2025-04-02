# import pyrealsense2 as rs
# import time
# pipeline = rs.pipeline()
# config = rs.config()
# config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
# config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
# cfg = pipeline.start(config)
# time.sleep(1)
# profile = cfg.get_stream(rs.stream.color)
# intr = profile.as_video_stream_profile().get_intrinsics()
# print(intr)  # 获取内参 width: 640, height: 480, ppx: 319.115, ppy: 234.382, fx: 597.267, fy: 597.267, model: Brown Conrady, coeffs: [0, 0, 0, 0, 0]
# time.sleep(1)
# profile = cfg.get_stream(rs.stream.depth)
# intr = profile.as_video_stream_profile().get_intrinsics()
# print(intr)  # 获取内参 width: 640, height: 480, ppx: 319.115, ppy: 234.382, fx: 597.267, fy: 597.267, model: Brown Conrady, coeffs: [0, 0, 0, 0, 0]


import pyrealsense2 as rs
import time

def print_intrinsics(resolution):
    pipeline = rs.pipeline()
    config = rs.config()
    
    # 配置分辨率参数
    width, height = resolution
    config.enable_stream(rs.stream.depth, width, height, rs.format.z16, 30)
    config.enable_stream(rs.stream.color, width, height, rs.format.rgb8, 30)
    
    try:
        cfg = pipeline.start(config)
        time.sleep(1)  # 等待相机初始化
        
        # 打印RGB内参
        color_profile = cfg.get_stream(rs.stream.color)
        color_intr = color_profile.as_video_stream_profile().get_intrinsics()
        print(f"\n{width}x{height} RGB内参:")
        print(f"ppx: {color_intr.ppx:.3f}, ppy: {color_intr.ppy:.3f}")
        print(f"fx: {color_intr.fx:.3f}, fy: {color_intr.fy:.3f}")
        
        # 打印深度内参
        depth_profile = cfg.get_stream(rs.stream.depth)
        depth_intr = depth_profile.as_video_stream_profile().get_intrinsics()
        print(f"\n{width}x{height} 深度内参:")
        print(f"ppx: {depth_intr.ppx:.3f}, ppy: {depth_intr.ppy:.3f}")
        print(f"fx: {depth_intr.fx:.3f}, fy: {depth_intr.fy:.3f}")
        
    finally:
        pipeline.stop()

# 检测640x480分辨率内参
print_intrinsics((640, 480))

# 检测1280x720分辨率内参
print_intrinsics((1280, 720))