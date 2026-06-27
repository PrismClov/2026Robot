import numpy as np
#坐标系前x方向，左y方向，垂直z方向
Target_Velocity_X = 0.0
Target_Velocity_Y = 0.0
Target_Omega = -0.1

#假设轮心距是1.0
Wheel_To_Core_Distance = 1.0
#舵轮方位角
#45度 135度 225度 315度
#归化到-pi ~ pi,即0.785 ~ 2.356 ~ -2.356 ~ -0.785
Steer_Azimuth = np.array([0.785, 2.356, -2.356, -0.785])  # 单位：弧度

target_angle = np.zeros(4)  # 初始化目标角度数组
for i in range(4):

    # 假设 Wheel_To_Core_Distance, Steer_Azimuth, Wheel_Force 都是 NumPy 数组
    x_i = Wheel_To_Core_Distance * np.cos(Steer_Azimuth[i])
    y_i = Wheel_To_Core_Distance * np.sin(Steer_Azimuth[i])

    v_wheel_x = Target_Velocity_X - Target_Omega * y_i
    v_wheel_y = Target_Velocity_Y + Target_Omega * x_i

    target_angle[i] = np.arctan2(v_wheel_y, v_wheel_x)

print ("Target Angles (radians):", target_angle)


import math

def normalize_angle_deg(angle):
    """
    归一化到 [-180, 180)
    """
    angle = math.fmod(angle + 180.0, 360.0)

    if angle < 0.0:
        angle += 360.0

    return angle - 180.0


def steer_optimize(now_angle, target_angle, target_velocity):
    """
    舵轮最短路径优化

    now_angle: 当前舵向角，单位 degree，可以是连续角度
    target_angle: 目标舵向角，单位 degree，通常来自 atan2
    target_velocity: 目标轮速

    return:
        target_angle_for_pid: 给 PID 用的连续目标角
        target_angle_normalized: 归一化后的显示角度
        target_velocity: 优化后的速度
        angle_error: 最短角度误差
    """

    # 原始误差
    angle_error = normalize_angle_deg(target_angle - now_angle)

    # 如果转向超过 90°，则目标角反向 180°，轮速取反
    if abs(angle_error) > 90.0:
        target_angle = target_angle + 180.0
        target_velocity = -target_velocity

        # 重新计算优化后的最短误差
        angle_error = normalize_angle_deg(target_angle - now_angle)

    # 给 PID 用的目标角：当前角度 + 最短误差
    target_angle_for_pid = now_angle + angle_error

    # 只用于显示或者保存的归一化角度
    target_angle_normalized = normalize_angle_deg(target_angle_for_pid)

    return target_angle_for_pid, target_angle_normalized, target_velocity, angle_error


Now_Angle = -180.0
Target_Angle = -45.0
Target_Velocity = 1.0

target_pid, target_norm, target_vel, error = steer_optimize(
    Now_Angle,
    Target_Angle,
    Target_Velocity
)

print("PID Target Angle:", target_pid)
print("Normalized Target Angle:", target_norm)
print("Optimized Velocity:", target_vel)
print("Optimized Error:", error)