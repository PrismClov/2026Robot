"""
Swerve chassis kinematics simulator — mirrors crt_chassis / dvc_swerve_module logic.

Verifies:
  1. Inverse kinematics (vx, vy, omega -> steer angles + wheel speeds)
  2. Forward kinematics (steer angles + wheel speeds -> vx, vy, omega)
  3. Steer angle optimization (<= 90 deg flip)
  4. Encoder CAN byte encode/decode
  5. DR16 remote control stick normalization
  6. Force distribution to wheels
  7. Dynamic closed-loop simulation with trajectory plotting

Run:  python steer_encoder_sim.py
"""

import math
import matplotlib
matplotlib.use('Agg')
# 配置中文字体
matplotlib.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'WenQuanYi Micro Hei', 'DejaVu Sans']
matplotlib.rcParams['axes.unicode_minus'] = False  # 负号显示
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ── Constants matching crt_chassis ──────────────────────────────────────────

WHEEL_RADIUS    = 0.0265   # m
WHEEL_REDUCTION = 7.5
WHEEL_TO_CORE   = [0.18, 0.18, 0.18, 0.18]  # m
# Steer azimuth angles: FL=FR=RL=RR positions (same as code)
STEER_AZIMUTH   = [-math.pi/4, -3*math.pi/4, 3*math.pi/4, math.pi/4]
WHEEL_LABELS    = ["FL", "FR", "RL", "RR"]

# ── Helpers mirroring drv_math ──────────────────────────────────────────────

def constrain(x, lo, hi):
    return max(lo, min(hi, x))

def modulus_normalization(x, mod):
    tmp = (x + mod / 2.0) % mod
    if tmp < 0:
        tmp += mod
    return tmp - mod / 2.0

def normalize_angle(x):
    return modulus_normalization(x, 2 * math.pi)

def normalize_angle_360(x):
    return modulus_normalization(x, 360.0)


# ── Chassis kinematics ──────────────────────────────────────────────────────

def kinematics_inverse(vx, vy, omega):
    """Inverse kinematics: chassis vel -> per-wheel target steer angle & velocity.

    Returns list of (steer_angle_rad, wheel_linear_vel_mps).
    """
    results = []
    for i in range(4):
        x_i = WHEEL_TO_CORE[i] * math.cos(STEER_AZIMUTH[i])
        y_i = WHEEL_TO_CORE[i] * math.sin(STEER_AZIMUTH[i])

        v_wx = vx - omega * y_i
        v_wy = vy + omega * x_i

        steer_angle = math.atan2(v_wy, v_wx)
        wheel_vel   = math.sqrt(v_wx**2 + v_wy**2)
        results.append((steer_angle, wheel_vel))
    return results

def kinematics_forward(steer_angles, motor_omega_radps):
    """Forward kinematics: per-wheel steer angle + motor rad/s -> chassis vel.

    motor_omega_radps: motor-side angular speed (before reduction).
    Returns (vx, vy, omega).
    """
    vx_sum = vy_sum = omega_sum = 0.0
    for i in range(4):
        wheel_radps = motor_omega_radps[i] / WHEEL_REDUCTION
        v_x = wheel_radps * WHEEL_RADIUS * math.cos(steer_angles[i])
        v_y = wheel_radps * WHEEL_RADIUS * math.sin(steer_angles[i])
        omega_c = wheel_radps * WHEEL_RADIUS * math.sin(steer_angles[i] - STEER_AZIMUTH[i]) / WHEEL_TO_CORE[i]
        vx_sum += v_x / 4.0
        vy_sum += v_y / 4.0
        omega_sum += omega_c / 4.0
    return vx_sum, vy_sum, omega_sum

def force_distribution(fx, fy, torque, steer_angles):
    """Distribute chassis force/torque to per-wheel longitudinal force.

    Matches crt_chassis Calculate():
      Wheel_Force[i] = FX*cos(sa) + FY*sin(sa) - T/dist*sin(azimuth - sa)
    """
    wheel_forces = []
    for i in range(4):
        sa = steer_angles[i]
        wf = (fx * math.cos(sa) + fy * math.sin(sa)
              - torque / WHEEL_TO_CORE[i] * math.sin(STEER_AZIMUTH[i] - sa))
        wheel_forces.append(wf)
    return wheel_forces


# ── Steer angle optimization ────────────────────────────────────────────────

def optimize_target(target_angle, current_angle):
    """Mirror Class_Swerve_Module::Optimize_Target.

    If error > 90 deg, flip direction. Returns (optimized_angle, flipped_bool).
    """
    error = normalize_angle(target_angle - current_angle)
    if abs(error) > math.pi * 0.5:
        opt_angle = normalize_angle(target_angle + math.pi)
        return opt_angle, True
    return target_angle, False


# ── Encoder simulator ────────────────────────────────────────────────────────

class SteerEncoderSim:
    """Simulates the CAN absolute encoder (0-65535 -> 0-360 deg).

    Mirrors dvc_steer_encoder: CAN frame has raw_angle_high/low,
    Data_Process maps to 0-360 deg, Get_Normalized_Angle wraps to [-180,180).
    """
    def __init__(self, zero_offset_rad=0.0):
        self.raw_angle_deg = 0.0
        self.zero_offset_rad = zero_offset_rad

    def set_angle_deg(self, deg):
        self.raw_angle_deg = deg % 360.0

    def get_normalized_deg(self):
        return normalize_angle_360(self.raw_angle_deg)

    def get_module_angle_rad(self):
        """What Swerve_Module::Get_Current_Angle returns."""
        return math.radians(self.get_normalized_deg()) - self.zero_offset_rad

    def to_can_bytes(self):
        """Encode to raw_angle_high / raw_angle_low (0-65535 range)."""
        raw = int(round(self.raw_angle_deg / 360.0 * 65535)) & 0xFFFF
        return raw >> 8, raw & 0xFF

    @staticmethod
    def from_can_bytes(high, low):
        raw = (high << 8) | low
        deg = raw / 65535.0 * 360.0
        enc = SteerEncoderSim()
        enc.set_angle_deg(deg)
        return enc


# ── DR16 remote simulator ───────────────────────────────────────────────────

class DR16RemoteSim:
    """Simulates DR16 remote stick channel encoding."""
    def __init__(self):
        self.ch0 = 1024  # right stick X
        self.ch1 = 1024  # right stick Y
        self.ch2 = 1024  # left stick Y
        self.ch3 = 1024  # left stick X
        self.rocker_offset = 1024.0
        self.rocker_num    = 660.0

    def set_left_stick(self, x_norm, y_norm):
        self.ch3 = int(self.rocker_offset + x_norm * self.rocker_num)
        self.ch2 = int(self.rocker_offset + y_norm * self.rocker_num)

    def set_right_stick(self, x_norm, y_norm):
        self.ch0 = int(self.rocker_offset + x_norm * self.rocker_num)
        self.ch1 = int(self.rocker_offset + y_norm * self.rocker_num)

    def get_normalized_left(self):
        return ((self.ch3 - self.rocker_offset) / self.rocker_num,
                (self.ch2 - self.rocker_offset) / self.rocker_num)

    def get_normalized_right(self):
        return ((self.ch0 - self.rocker_offset) / self.rocker_num,
                (self.ch1 - self.rocker_offset) / self.rocker_num)

    def to_channels(self):
        return self.ch0, self.ch1, self.ch2, self.ch3


# ── Simple PID ──────────────────────────────────────────────────────────────

class SimplePID:
    def __init__(self, kp, ki, kd, out_max=1.0, i_max=1.0, dt=0.001):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_max = out_max
        self.i_max = i_max
        self.dt = dt
        self.target = 0.0
        self.now = 0.0
        self.integral = 0.0
        self.prev_err = 0.0
        self._out = 0.0

    def set(self, target, now):
        self.target = target
        self.now = now
        err = target - now
        self.integral += err * self.dt
        self.integral = constrain(self.integral, -self.i_max, self.i_max)
        derivative = (err - self.prev_err) / self.dt if self.dt > 0 else 0
        self.prev_err = err
        self._out = constrain(self.kp * err + self.ki * self.integral + self.kd * derivative,
                              -self.out_max, self.out_max)

    def out(self):
        return self._out


# ── Swerve module simulator ─────────────────────────────────────────────────

class SwerveModuleSim:
    """One corner: encoder + steer servo (first-order) + wheel motor (first-order)."""
    def __init__(self, steer_tau=0.03, wheel_tau=0.05, zero_offset_rad=0.0):
        self.encoder = SteerEncoderSim(zero_offset_rad)
        self.steer_tau = steer_tau    # time constant [s] for servo
        self.wheel_tau = wheel_tau
        self.target_angle = 0.0
        self.target_force = 0.0
        self.target_wheel_speed = 0.0  # motor rad/s, from inverse kinematics
        self.wheel_speed = 0.0          # motor rad/s, actual

    def get_current_angle_rad(self):
        return self.encoder.get_module_angle_rad()

    def set_target_angle(self, angle_rad):
        self.target_angle = normalize_angle(angle_rad)

    def set_target_force(self, force_n):
        self.target_force = force_n

    def set_target_wheel_speed(self, motor_radps):
        self.target_wheel_speed = motor_radps

    def step(self, dt=0.001):
        # Steer: first-order response toward target
        current = self.get_current_angle_rad()
        angle_diff = normalize_angle(self.target_angle - current)
        new_angle = math.degrees(current + angle_diff / self.steer_tau * dt)
        self.encoder.set_angle_deg(new_angle)

        # Wheel: first-order response toward target speed
        self.wheel_speed += (self.target_wheel_speed - self.wheel_speed) / self.wheel_tau * dt


# ── Full chassis simulator ───────────────────────────────────────────────────

class ChassisSim:
    """Full chassis simulation matching Class_Chassis control loop."""
    def __init__(self):
        self.modules = [SwerveModuleSim() for _ in range(4)]
        self.vx = self.vy = self.omega = 0.0
        self.target_vx = self.target_vy = self.target_omega = 0.0
        self.fx = self.fy = self.torque = 0.0
        self.wheel_forces = [0.0] * 4
        self.target_steer_angles = [0.0] * 4
        self.chassis_x = self.chassis_y = self.chassis_yaw = 0.0

        # PID gains (matching crt_chassis Init: kp=10, i=0, d=0)
        self.pid_vx = SimplePID(10.0, 0.0, 0.0, out_max=18.0, i_max=30.0)
        self.pid_vy = SimplePID(10.0, 0.0, 0.0, out_max=18.0, i_max=30.0)
        self.pid_omega = SimplePID(10.0, 0.0, 0.0, out_max=10.0, i_max=10.0)

    def set_target(self, vx, vy, omega):
        self.target_vx = vx
        self.target_vy = vy
        self.target_omega = omega

    def step(self, dt=0.001):
        # ── Calculate: PID on chassis velocities ──
        self.pid_vx.set(self.target_vx, self.vx)
        self.pid_vy.set(self.target_vy, self.vy)
        self.pid_omega.set(self.target_omega, self.omega)
        self.fx     = self.pid_vx.out()
        self.fy     = self.pid_vy.out()
        self.torque = self.pid_omega.out()

        # ── Force distribution to wheels ──
        cur_angles = [self.modules[i].get_current_angle_rad() for i in range(4)]
        self.wheel_forces = force_distribution(self.fx, self.fy, self.torque, cur_angles)

        # ── Inverse kinematics ──
        inv = kinematics_inverse(self.target_vx, self.target_vy, self.target_omega)
        for i in range(4):
            sa, wheel_lin_vel = inv[i]
            self.modules[i].set_target_angle(sa)
            self.modules[i].set_target_force(self.wheel_forces[i])
            # Convert linear wheel speed to motor-side rad/s
            wheel_radps = wheel_lin_vel / WHEEL_RADIUS
            motor_radps = wheel_radps * WHEEL_REDUCTION
            self.modules[i].set_target_wheel_speed(motor_radps)
            self.target_steer_angles[i] = sa

        # ── Module step ──
        for i in range(4):
            self.modules[i].step(dt)

        # ── Self-resolution: read back chassis velocity from wheels ──
        steer_angles = [self.modules[i].get_current_angle_rad() for i in range(4)]
        wheel_omega  = [self.modules[i].wheel_speed for i in range(4)]
        self.vx, self.vy, self.omega = kinematics_forward(steer_angles, wheel_omega)

        # ── Integrate chassis pose ──
        self.chassis_x   += self.vx * dt
        self.chassis_y   += self.vy * dt
        self.chassis_yaw += self.omega * dt


# ── Test scenarios ───────────────────────────────────────────────────────────

def test_kinematics_roundtrip():
    """Verify inverse -> forward recovers original velocities."""
    print("=" * 60)
    print("TEST 1: Kinematics roundtrip (inverse -> forward)")
    print("=" * 60)
    test_cases = [
        (1.0, 0.0, 0.0),     # pure forward
        (0.0, 1.0, 0.0),     # pure strafe
        (0.0, 0.0, 1.0),     # pure rotation
        (1.0, 1.0, 0.0),     # diagonal
        (1.0, 0.0, 2.0),     # forward + spin
        (-0.5, 0.8, 1.5),    # mixed
        (0.0, 0.0, 0.0),     # standstill
        (2.0, -1.0, -3.0),   # extreme
    ]
    all_pass = True
    for vx, vy, omega in test_cases:
        inv = kinematics_inverse(vx, vy, omega)
        steer_angles = [a[0] for a in inv]
        # Convert linear vel to motor rad/s
        wheel_omega = []
        for _, wheel_vel in inv:
            wheel_radps = wheel_vel / WHEEL_RADIUS
            motor_radps = wheel_radps * WHEEL_REDUCTION
            wheel_omega.append(motor_radps)
        rvx, rvy, romega = kinematics_forward(steer_angles, wheel_omega)
        ok = (abs(rvx - vx) < 1e-6 and abs(rvy - vy) < 1e-6 and
              abs(romega - omega) < 1e-6)
        status = "PASS" if ok else "FAIL"
        if not ok:
            all_pass = False
        print(f"  ({vx:6.2f}, {vy:6.2f}, {omega:6.2f}) -> "
              f"rev({rvx:6.4f}, {rvy:6.4f}, {romega:6.4f}) [{status}]")
    print(f"  => {'ALL PASS' if all_pass else 'SOME FAILED'}\n")
    return all_pass

def test_steer_optimization():
    """Test steer flip logic (error > 90 deg -> flip 180)."""
    print("=" * 60)
    print("TEST 2: Steer angle optimization")
    print("=" * 60)
    cases = [
        (0.0,    0.0,   False),
        (0.5,    0.0,   False),
        (1.0,    0.0,   False),
        (1.6,    0.0,   True),
        (math.pi, 0.0,  True),
        (-math.pi/2, math.pi/4, True),
        (2.0,    -1.0,  True),   # ~172° diff -> flip
        (2.8,    -2.0,  False),  # ~85° diff -> no flip (just under 90)
    ]
    all_pass = True
    for target, current, expect_flip in cases:
        opt, flipped = optimize_target(target, current)
        err = normalize_angle(opt - current)
        ok = abs(err) <= math.pi / 2 + 1e-9 and (flipped == expect_flip)
        if not ok:
            all_pass = False
        print(f"  tgt={math.degrees(target):7.1f} cur={math.degrees(current):7.1f} "
              f"-> opt={math.degrees(opt):7.1f} flip={flipped} "
              f"err={math.degrees(err):6.1f} [{('PASS' if ok else 'FAIL')}]")
    print(f"  => {'ALL PASS' if all_pass else 'SOME FAILED'}\n")
    return all_pass

def test_encoder_codec():
    """Verify encoder CAN byte encode/decode roundtrip."""
    print("=" * 60)
    print("TEST 3: Encoder CAN byte codec roundtrip")
    print("=" * 60)
    test_angles = [0, 45, 90, 135, 180, 225, 270, 315, 359.9]
    all_pass = True
    for deg in test_angles:
        enc = SteerEncoderSim()
        enc.set_angle_deg(deg)
        hi, lo = enc.to_can_bytes()
        decoded = SteerEncoderSim.from_can_bytes(hi, lo)
        recovered = decoded.raw_angle_deg
        err = abs(recovered - deg % 360)
        ok = err < 0.01
        if not ok:
            all_pass = False
        print(f"  {deg:6.1f}° -> CAN({hi:3d},{lo:3d}) -> {recovered:8.3f}° "
              f"err={err:.4f}° [{('PASS' if ok else 'FAIL')}]")
    print(f"  => {'ALL PASS' if all_pass else 'SOME FAILED'}\n")
    return all_pass

def test_remote_codec():
    """Test DR16 remote stick channel encoding roundtrip."""
    print("=" * 60)
    print("TEST 4: DR16 Remote stick normalization roundtrip")
    print("=" * 60)
    remote = DR16RemoteSim()
    cases = [
        (0.0, 0.0, "center"),
        (1.0, 0.0, "right"),
        (-1.0, 1.0, "top-left"),
        (0.5, -0.3, "partial"),
    ]
    all_pass = True
    for x, y, desc in cases:
        remote.set_left_stick(x, y)
        rx, ry = remote.get_normalized_left()
        ch = remote.to_channels()
        ok = abs(rx - x) < 1e-6 and abs(ry - y) < 1e-6
        if not ok:
            all_pass = False
        print(f"  {desc:12s} ({x:5.2f},{y:5.2f}) -> ch=({ch[0]:4d},{ch[2]:4d}) "
              f"-> ({rx:.4f},{ry:.4f}) [{('PASS' if ok else 'FAIL')}]")
    print(f"  => {'ALL PASS' if all_pass else 'SOME FAILED'}\n")
    return all_pass

def test_force_distribution():
    """Print force distribution for known cases."""
    print("=" * 60)
    print("TEST 5: Force distribution to wheels")
    print("=" * 60)
    steer_angles = STEER_AZIMUTH[:]
    cases = [
        (10.0, 0.0, 0.0, "pure FX=10N"),
        (0.0, 10.0, 0.0, "pure FY=10N"),
        (0.0, 0.0, 5.0,  "pure T=5Nm"),
        (10.0, 10.0, 3.0,"combined"),
    ]
    for fx, fy, t, desc in cases:
        wf = force_distribution(fx, fy, t, steer_angles)
        print(f"  {desc:20s} FX={fx:5.1f} FY={fy:5.1f} T={t:5.1f}")
        for i in range(4):
            print(f"    {WHEEL_LABELS[i]}: {wf[i]:7.3f} N")
    print()

def test_chassis_dynamic():
    """Dynamic simulation with separate tests for VX, VY, Omega inputs."""
    print("=" * 60)
    print("TEST 6: Dynamic chassis simulation")
    print("=" * 60)

    dt = 0.001  # 1 ms control
    total_time = 3.0
    n_steps = int(total_time / dt)
    timeline = np.linspace(0, total_time, n_steps)
    t_ms = timeline * 1000

    # Three separate tests: VX only, VY only, Omega only
    test_names = ["X 方向输入", "Y 方向输入", "Omega 输入"]

    fig = plt.figure(figsize=(16, 12))

    for test_idx in range(3):
        # Create simulation for this test
        sim = ChassisSim()
        rec = {k: np.zeros(n_steps) for k in
               ['tvx', 'tvy', 'tom', 'nvx', 'nvy', 'nom', 'cx', 'cy', 'cyaw']}

        # Command function for this test
        if test_idx == 0:  # X direction
            def get_cmd(t):
                return (1.0, 0.0, 0.0) if t < 1.0 else (0.0, 0.0, 0.0)
        elif test_idx == 1:  # Y direction
            def get_cmd(t):
                return (0.0, 1.0, 0.0) if t < 1.0 else (0.0, 0.0, 0.0)
        else:  # Omega
            def get_cmd(t):
                return (0.0, 0.0, 2.0) if t < 1.0 else (0.0, 0.0, 0.0)

        print(f"  Running {test_names[test_idx]}...")
        for step in range(n_steps):
            t = step * dt
            cmd = get_cmd(t)
            sim.set_target(*cmd)
            sim.step(dt)

            rec['tvx'][step] = cmd[0]; rec['tvy'][step] = cmd[1]; rec['tom'][step] = cmd[2]
            rec['nvx'][step] = sim.vx; rec['nvy'][step] = sim.vy; rec['nom'][step] = sim.omega
            rec['cx'][step] = sim.chassis_x; rec['cy'][step] = sim.chassis_y
            rec['cyaw'][step] = sim.chassis_yaw

        # Print steady-state error
        idx = slice(int(0.8 / dt), int(0.9 / dt))
        if test_idx == 2:  # Omega
            target, mean = 2.0, np.mean(rec['nom'][idx])
        elif test_idx == 0:  # X
            target, mean = 1.0, np.mean(rec['nvx'][idx])
        else:  # Y
            target, mean = 1.0, np.mean(rec['nvy'][idx])
        err_pct = abs(mean - target) / abs(target) * 100 if target != 0 else abs(mean) * 100
        print(f"    {test_names[test_idx]}: target={target}, mean={mean:.4f}, err={err_pct:.1f}%")
        print(f"    Final pose: x={sim.chassis_x:.3f} y={sim.chassis_y:.3f} yaw={math.degrees(sim.chassis_yaw):.1f}°")

        base_col = test_idx * 4 + 1

        # Plot 1: Remote input
        ax1 = plt.subplot(3, 4, base_col)
        if test_idx == 0:
            ax1.plot(t_ms, rec['tvx'], 'r-', lw=2, label='目标 VX')
            ax1.set_ylabel('VX (m/s)')
        elif test_idx == 1:
            ax1.plot(t_ms, rec['tvy'], 'g-', lw=2, label='目标 VY')
            ax1.set_ylabel('VY (m/s)')
        else:
            ax1.plot(t_ms, rec['tom'], 'b-', lw=2, label='目标 Omega')
            ax1.set_ylabel('Omega (rad/s)')
        ax1.set_title(f'遥控器输入 ({test_names[test_idx]})')
        ax1.set_ylim(-0.2, 2.0)
        ax1.set_xlim(0, total_time * 1000)
        ax1.grid(True, alpha=0.3); ax1.legend(fontsize=8)

        # Plot 2: Velocity tracking
        ax2 = plt.subplot(3, 4, base_col + 1)
        if test_idx == 0:
            ax2.plot(t_ms, rec['tvx'], 'r-', lw=1.5, label='目标')
            ax2.plot(t_ms, rec['nvx'], 'b-', lw=1, alpha=0.8, label='实测')
            ax2.set_ylabel('VX (m/s)')
        elif test_idx == 1:
            ax2.plot(t_ms, rec['tvy'], 'g-', lw=1.5, label='目标')
            ax2.plot(t_ms, rec['nvy'], 'b-', lw=1, alpha=0.8, label='实测')
            ax2.set_ylabel('VY (m/s)')
        else:
            ax2.plot(t_ms, rec['tom'], 'b-', lw=1.5, label='目标')
            ax2.plot(t_ms, rec['nom'], 'r-', lw=1, alpha=0.8, label='实测')
            ax2.set_ylabel('Omega (rad/s)')
        ax2.set_title('速度跟踪')
        ax2.set_ylim(-0.2, 2.0)
        ax2.set_xlim(0, total_time * 1000)
        ax2.grid(True, alpha=0.3); ax2.legend(fontsize=8)

        # Plot 3: Trajectory
        ax3 = plt.subplot(3, 4, base_col + 2)
        ax3.plot(rec['cx'], rec['cy'], 'b-', lw=1.5, label='轨迹')
        ax3.plot(rec['cx'][0], rec['cy'][0], 'go', ms=8, label='起点')
        ax3.plot(rec['cx'][-1], rec['cy'][-1], 'ro', ms=8, label='终点')
        ax3.set_xlabel('X (m)'); ax3.set_ylabel('Y (m)')
        ax3.set_title('运动轨迹')
        traj_range = 1.5
        ax3.set_xlim(-traj_range, traj_range); ax3.set_ylim(-traj_range, traj_range)
        ax3.set_aspect('equal'); ax3.grid(True, alpha=0.3); ax3.legend(fontsize=8)

        # Plot 4: Heading angle
        ax4 = plt.subplot(3, 4, base_col + 3)
        ax4.plot(t_ms, np.degrees(rec['cyaw']), 'b-', lw=1.5)
        ax4.set_xlabel('时间 (ms)')
        ax4.set_ylabel('航向角 (°)')
        ax4.set_title('底盘航向')
        ax4.set_xlim(0, total_time * 1000)
        ax4.grid(True, alpha=0.3)

    plt.suptitle('舵轮底盘运动学仿真 - 独立输入测试', fontsize=14, y=0.98)
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig('steer_encoder_sim.png', dpi=150, bbox_inches='tight')
    print("  Plot saved to: steer_encoder_sim.png")
    plt.close()

def test_encoder_steering_tracking():
    """Verify encoder tracks steer servo command in simulation."""
    print("=" * 60)
    print("TEST 7: Encoder steering tracking")
    print("=" * 60)
    mod = SwerveModuleSim(zero_offset_rad=0.0)
    targets = [0, math.pi/4, math.pi/2, math.pi, -math.pi/2, -math.pi/4]
    dt = 0.001

    for target in targets:
        mod.set_target_angle(target)
        for _ in range(200):
            mod.step(dt)  # let settle for 200ms
        current = mod.get_current_angle_rad()
        err = normalize_angle(current - target)
        ok = abs(err) < 0.05
        print(f"  Target {math.degrees(target):7.1f}° -> "
              f"Enc {math.degrees(current):7.1f}°  err={math.degrees(err):6.2f}° "
              f"[{'PASS' if ok else 'WARN'}]")
    print()

def main():
    print("Swerve Chassis Kinematics Simulator")
    print("Mirrors: crt_chassis / dvc_swerve_module / dvc_steer_encoder / dvc_dr16\n")

    test_kinematics_roundtrip()
    test_steer_optimization()
    test_encoder_codec()
    test_remote_codec()
    test_force_distribution()
    test_encoder_steering_tracking()
    test_chassis_dynamic()

    print("=" * 60)
    print("All tests complete!")
    print("=" * 60)

if __name__ == "__main__":
    main()
