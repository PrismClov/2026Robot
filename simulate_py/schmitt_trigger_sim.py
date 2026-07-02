"""
鼠标交互式仿真：舵轮施密特触发器
- 鼠标在画布上移动，解析为底盘速度方向 (Vx, Vy)
- 实时显示四个轮的目标舵向角、施密特触发输出、翻转状态
- 在 ±PI/2 边界附近抖动时观察施密特触发的抑制效果

使用方法：
    python schmitt_trigger_sim.py
    用鼠标在左图拖动红点，观察右侧数据变化
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow
import matplotlib.patches as mpatches
import matplotlib
import math
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'WenQuanYi Micro Hei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False

PI = np.pi
hysteresis = 0.0995  # 约5.7°迟滞阈值
half_pi = PI / 2.0

# 轮方位角 (与 cpp 代码一致)
STEER_AZIMUTH = np.array([0.785, 2.356, -2.356, -0.785])  # 45°, 135°, -135°, -45°
WHEEL_NAMES = ["轮0 (45°)", "轮1 (135°)", "轮2 (-135°)", "轮3 (-45°)"]
WHEEL_COLORS = ["#e41a1c", "#377eb8", "#4daf4a", "#984ea3"]

# 施密特触发器状态 (per wheel)
flip_state = np.zeros(4, dtype=bool)


def normalize_angle(angle):
    """
    等价于 C++ 代码的:
        tmp_delta_angle = fmod(tmp_delta_angle, 2.0f * PI);
        tmp_delta_angle = Math_Modulus_Normalization(tmp_delta_angle, 2.0f * PI);
    即归一化到 (-PI, PI]
    """
    delta = math.fmod(angle, 2.0 * PI)
    return math.fmod(delta + PI, 2.0 * PI) - PI


def schmitt_trigger_logic(now_angle, target_angle, flip_state_i):
    """
    施密特触发器逻辑
    flip_state_i: 当前翻转状态 (True=已翻转走优弧)
    返回: (目标角度, 翻转状态)
    """
    delta = normalize_angle(target_angle - now_angle)

    if not flip_state_i:
        if delta > half_pi + hysteresis:
            new_angle = normalize_angle(delta + PI) + now_angle
            return new_angle, True
        else:
            return delta + now_angle, False
    else:
        if delta < -half_pi - hysteresis:
            return delta + now_angle, False
        else:
            new_angle = normalize_angle(delta + PI) + now_angle
            return new_angle, True


def compute_wheel_target_angles(vx, vy, omega=0.0):
    """运动学逆解算：计算四个轮的目标舵向角"""
    target_angles = np.zeros(4)
    for i in range(4):
        x_i = np.cos(STEER_AZIMUTH[i])
        y_i = np.sin(STEER_AZIMUTH[i])
        v_wheel_x = vx - omega * y_i
        v_wheel_y = vy + omega * x_i
        target_angles[i] = np.arctan2(v_wheel_y, v_wheel_x)
    return target_angles


class MouseSimulator:
    def __init__(self):
        self.vx = 1.0
        self.vy = 0.0
        self.omega = 0.0
        self.now_steer_angle = np.zeros(4)

        self.fig, self.axes = plt.subplots(1, 2, figsize=(14, 7),
                                            gridspec_kw={'width_ratios': [1, 1.3]})
        self.fig.canvas.mpl_connect('motion_notify_event', self.on_mouse_move)
        self.fig.canvas.mpl_connect('button_press_event', self.on_mouse_press)
        self.fig.canvas.mpl_connect('button_release_event', self.on_mouse_release)
        self.dragging = False

        self._setup_left_panel()
        self._setup_right_panel()
        self.update()

    def _setup_left_panel(self):
        """左侧：速度矢量图"""
        self.ax_left = self.axes[0]
        self.ax_left.set_xlim(-2.5, 2.5)
        self.ax_left.set_ylim(-2.5, 2.5)
        self.ax_left.set_aspect('equal')
        self.ax_left.set_xlabel('Vx (m/s)')
        self.ax_left.set_ylabel('Vy (m/s)')
        self.ax_left.set_title('拖动红点改变速度方向\n(距离=速度大小)')
        self.ax_left.axhline(0, color='gray', linewidth=0.5)
        self.ax_left.axvline(0, color='gray', linewidth=0.5)
        self.ax_left.set_facecolor('#1e1e1e')
        self.ax_left.set_facecolor('#f0f0f0')

        # 速度圆
        theta = np.linspace(0, 2 * PI, 200)
        self.ax_left.plot(np.cos(theta), np.sin(theta), 'k--', alpha=0.2)
        self.ax_left.text(0, 0, 'O', ha='center', va='center', fontsize=10, color='gray')

        # 速度箭头
        self.arrow_patch = FancyArrow(0, 0, 1, 0, width=0.12, head_width=0.3,
                                       head_length=0.1, fc='red', ec='darkred')
        self.ax_left.add_patch(self.arrow_patch)

        # 鼠标拖动点
        self.drag_point, = self.ax_left.plot(1, 0, 'ro', markersize=10, zorder=10)

        # 四个轮方位线
        for i, (az, name) in enumerate(zip(STEER_AZIMUTH, WHEEL_NAMES)):
            dx = 1.6 * np.cos(az)
            dy = 1.6 * np.sin(az)
            self.ax_left.annotate('', xy=(dx, dy), xytext=(0, 0),
                                  arrowprops=dict(arrowstyle='->', color=WHEEL_COLORS[i], lw=1.5))
            self.ax_left.text(dx * 1.1, dy * 1.1, f'{name}\n{az*180/PI:.0f}°',
                             ha='center', va='center', fontsize=7, color=WHEEL_COLORS[i])

    def _setup_right_panel(self):
        """右侧：四轮角度数据"""
        self.ax_right = self.axes[1]
        self.ax_right.set_xlim(-1, 4)
        self.ax_right.set_ylim(-200, 200)
        self.ax_right.set_xlabel('')
        self.ax_right.set_ylabel('角度 (°)')
        self.ax_right.set_title('四轮目标角度 (蓝=原始目标, 红=施密特触发输出)')
        self.ax_right.axhline(90, color='orange', linestyle=':', alpha=0.6, label='+90° 阈值')
        self.ax_right.axhline(-90, color='orange', linestyle=':', alpha=0.6, label='-90° 阈值')
        self.ax_right.axhline(90 + 5.7, color='green', linestyle=':', alpha=0.4, label='上迟滞线 95.7°')
        self.ax_right.axhline(-90 - 5.7, color='green', linestyle=':', alpha=0.4, label='下迟滞线 -95.7°')
        self.ax_right.legend(fontsize=7, loc='upper right')

        self.wheel_lines_target = []
        self.wheel_lines_out = []
        self.wheel_texts = []
        self.wheel_flip_texts = []

        for i in range(4):
            line_target, = self.ax_right.plot([], [], 'o-', color=WHEEL_COLORS[i],
                                               linewidth=1.5, markersize=4, label=f'{WHEEL_NAMES[i]} 原始')
            line_out, = self.ax_right.plot([], [], 's--', color=WHEEL_COLORS[i],
                                             linewidth=2, markersize=6, alpha=0.8,
                                             markerfacecolor='none', label=f'{WHEEL_NAMES[i]} 施密特')
            self.wheel_lines_target.append(line_target)
            self.wheel_lines_out.append(line_out)

            # 每个轮的角度标签
            text = self.ax_right.text(3.5, 0, '', fontsize=8, color=WHEEL_COLORS[i],
                                       va='center')
            self.wheel_texts.append(text)

            flip_text = self.ax_right.text(3.85, 0, '', fontsize=8,
                                            va='center', color='red')
            self.wheel_flip_texts.append(flip_text)

        self.ax_right.set_xticks([])
        self.info_text = self.ax_right.text(0.02, 0.02, '', transform=self.ax_right.transAxes,
                                             fontsize=9, va='bottom',
                                             bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))

    def on_mouse_press(self, event):
        if event.inaxes == self.ax_left:
            self.dragging = True
            self.update_from_event(event)

    def on_mouse_release(self, event):
        self.dragging = False

    def on_mouse_move(self, event):
        if self.dragging and event.inaxes == self.ax_left:
            self.update_from_event(event)

    def update_from_event(self, event):
        if event.xdata is None or event.ydata is None:
            return
        self.vx = np.clip(event.xdata, -2, 2)
        self.vy = np.clip(event.ydata, -2, 2)

        # 更新箭头
        self.arrow_patch.remove()
        speed = np.hypot(self.vx, self.vy)
        if speed > 0.05:
            self.arrow_patch = FancyArrow(0, 0, self.vx, self.vy, width=0.1,
                                           head_width=0.25, head_length=0.08,
                                           fc='red', ec='darkred')
            self.ax_left.add_patch(self.arrow_patch)
            self.drag_point.set_data([self.vx], [self.vy])
        else:
            self.drag_point.set_data([0], [0])

        self.update()

    def update(self):
        """根据当前速度更新所有数据并重绘"""
        speed = np.hypot(self.vx, self.vy)

        # 计算四个轮目标角度
        target_angles = compute_wheel_target_angles(self.vx, self.vy, self.omega)

        global flip_state
        out_angles = np.zeros(4)
        for i in range(4):
            out_angle, flip_state[i] = schmitt_trigger_logic(
                self.now_steer_angle[i], target_angles[i], flip_state[i]
            )
            out_angles[i] = out_angle

        # 更新左图标题
        self.ax_left.set_title(
            f'拖动红点改变速度方向\nVx={self.vx:.2f}  Vy={self.vy:.2f}  |V|={speed:.2f}',
            fontsize=10
        )

        # 更新右图数据
        for i in range(4):
            # 角度条 (归一化到度)
            tgt_deg = target_angles[i] * 180 / PI
            out_deg = out_angles[i] * 180 / PI

            # 用 bar 代替 line
            x_pos = i
            self.wheel_lines_target[i].set_data([x_pos], [tgt_deg])
            self.wheel_lines_out[i].set_data([x_pos], [out_deg])

            # 更新文字标签
            delta_deg = tgt_deg
            self.wheel_texts[i].set_position((3.5, tgt_deg))
            self.wheel_texts[i].set_text(
                f'目标:{tgt_deg:+7.1f}°\n'
                f'施密特:{out_deg:+7.1f}°'
            )
            self.wheel_texts[i].set_fontsize(7)

            flip_label = "优弧" if flip_state[i] else "劣弧"
            self.wheel_flip_texts[i].set_position((3.9, out_deg))
            self.wheel_flip_texts[i].set_text(flip_label)
            self.wheel_flip_texts[i].set_color('red' if flip_state[i] else 'gray')

        # 更新 info
        self.info_text.set_text(
            f'速度: Vx={self.vx:+.2f} m/s  Vy={self.vy:+.2f} m/s\n'
            f'施密特迟滞: {hysteresis*180/PI:.1f}°  阈值: ±{half_pi*180/PI:.1f}° ±{hysteresis*180/PI:.1f}°'
        )

        self.fig.canvas.draw_idle()
        self.fig.canvas.flush_events()

    def show(self):
        plt.tight_layout()
        plt.show()


if __name__ == "__main__":
    sim = MouseSimulator()
    sim.show()