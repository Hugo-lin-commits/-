"""全部可调参数集中在这一份文件。

改参分级
--------
L0  不上天就错：舵机符号、混控编号、相机内参、IMU 安装
L1  第一发就能飞：串级 PID、限幅、时序、弹目初值
L2  有气动 ident / CFD 之后：三回路极点配置用的气动导数
L3  导引：导航比、视线滤波、ISMCG
L4  耦合前馈、动压调度（西交实测也需要）

西交开源里的数值是「他们那只旋成体 + X 尾」在某一标称点冻结出来的，
不能当你们三角一体翼的默认增益。结构可以照搬，数字必须重测。
"""

from __future__ import annotations

from dataclasses import dataclass, field


# ---------------------------------------------------------------------------
# L0 硬件 / 安装 —— 台架上必须先校完
# ---------------------------------------------------------------------------

@dataclass
class HardwareParams:
    """后视、弹头朝远离你：X 四片一体翼。编号与设计计算器一致。

    后视逆时针，1 = 右上 45°::

        2(左上)     1(右上)
        3(左下)     4(右下)

    混控（等效舵偏 δp/δr/δy → 四片后缘舵）::

        δ1 = δp + δr + δy
        δ2 = δp - δr - δy
        δ3 = δp - δr + δy
        δ4 = δp + δr - δy

    正舵约定（改符号前不要动混控公式）:
      +δp  四片同向，产生抬头力矩
      +δr  右侧(1,4) 与左侧(2,3) 反向，产生右滚
      +δy  X 相位，产生右偏航
    """

    # 每路舵机安装反向：台架上给 +δp，若实际低头则把对应通道乘 -1
    servo_sign: tuple[float, float, float, float] = (1.0, 1.0, 1.0, 1.0)
    servo_trim_deg: tuple[float, float, float, float] = (0.0, 0.0, 0.0, 0.0)
    delta_max_deg: float = 18.0
    pwm_us_center: float = 1500.0
    pwm_us_per_deg: float = 11.1  # 约 500us / 45deg，按舵机说明书改

    # IMU：TDK ICM-42688。下面 sign 是弹体系微调；芯片→弹体映射见 imu_chip_idx
    imu_gyro_sign: tuple[float, float, float] = (1.0, 1.0, 1.0)  # p, q, r
    imu_accel_sign: tuple[float, float, float] = (1.0, 1.0, 1.0)
    imu_model: str = "icm42688"
    gyro_fs_dps: float = 2000.0  # 飞镖发射扰动大，不要用 MaixPy 默认 ±256 dps
    accel_fs_g: float = 16.0  # 出膛过载常 >8 g，不要用 ±2 g
    imu_odr_hz: float = 1000.0
    # 芯片轴 → 弹体 pqr / ax ay az。默认：MaixCAM2 镜头朝前，芯片与板对齐
    # body_x=chip_y, body_y=chip_x, body_z=-chip_z
    imu_chip_idx: tuple[int, int, int] = (1, 0, 2)
    imu_chip_sign: tuple[float, float, float] = (1.0, 1.0, -1.0)

    # MaixCAM2 窗口后的像素内参（默认按 OS04D10 水平 ~81°、640 宽估算）
    # 必须用棋盘格或单灯标定，不要当最终值
    cam_width: float = 640.0
    cam_height: float = 360.0
    cam_fx: float = 375.0
    cam_fy: float = 378.0
    cam_cx: float = 320.0
    cam_cy: float = 180.0

    # MaixCAM2 SPI2 默认脚：B20 SCK / B18 MOSI / B19 MISO / B21 CS1
    spi_id: int = 2
    spi_freq: int = 8_000_000
    # 四路舵机 (pin, pwm_id)，顺序=混控 1~4。必须对照你们分电板丝印改
    servo_pwm: tuple[tuple[str, int], tuple[str, int], tuple[str, int], tuple[str, int]] = (
        ("A31", 7),
        ("A30", 6),
        ("A29", 5),
        ("A28", 4),
    )
    # 相机相对弹体：+1 表示像素增大对应弹体 +yaw / +pitch
    los_yaw_sign: float = -1.0
    los_pitch_sign: float = 1.0


# ---------------------------------------------------------------------------
# L0/L2 气动冻结系数 —— 西交用 CFD 在发射标称点冻结；你们必须重做
# ---------------------------------------------------------------------------

@dataclass
class AeroFrozen:
    """短周期线性化系数，定义见西交 2.4.4.1。

    西交测量值仅作数量级参考（旋成体 X 尾，有涵道推力项）::

        a_omega=0.209  a_alpha=0.091  a_delta=6.446
        b_alpha=0.358  b_delta=0.0835
        d_omega=0.043  d_delta=13.283

    三角一体翼、无动力时：
      - P=0，所以 b_alpha = C_y^alpha * q * S / (m V)，没有 +P/(mV)
      - 后缘升降副翼舵效通常弱于全动尾舵，a_delta / b_delta 会更小
      - 大攻角涡升力让 a_alpha 非线性更强，冻结点建议取下降段 12 m/s 而不是出膛点
    """

    # 俯仰/偏航（轴对称 X，两通道共用）
    a_omega: float = 0.20  # 阻尼  1/s   —— CFD 或角速度衰减辨识
    a_alpha: float = 0.10  # 静稳定 1/s² —— 与静稳定裕度同一符号约定
    a_delta: float = 5.00  # 舵效  1/s² —— 地面/风洞/CFD
    b_alpha: float = 0.30  # 攻角→弹道倾角速率  1/s
    b_delta: float = 0.06  # 舵偏→过载          1/s
    # 滚转
    d_omega: float = 0.05  # 滚转阻尼 1/s
    d_delta: float = 10.0  # 滚转舵效 1/s²

    v_ref: float = 12.0  # 冻结点空速 m/s（建议下降段，不是 V0）
    q_ref_scale: bool = False  # True: 增益随 (V/v_ref)^2 缩放；先关，数据够再开


# ---------------------------------------------------------------------------
# L1 串级 PID —— 没有 CFD 时的第一套能飞控制器
# ---------------------------------------------------------------------------

@dataclass
class CascadePidGains:
    kp: float
    ki: float
    kd: float
    i_lim: float
    out_lim: float


@dataclass
class CascadeParams:
    """外环角度、内环角速度。单位：rad / rad/s / 等效舵偏 rad。"""

    roll_ang: CascadePidGains = field(
        default_factory=lambda: CascadePidGains(4.0, 0.4, 0.05, 0.4, 2.5)
    )
    roll_rate: CascadePidGains = field(
        default_factory=lambda: CascadePidGains(0.35, 0.0, 0.0, 0.2, 0.35)
    )
    pitch_ang: CascadePidGains = field(
        default_factory=lambda: CascadePidGains(3.5, 0.25, 0.04, 0.3, 1.5)
    )
    pitch_rate: CascadePidGains = field(
        default_factory=lambda: CascadePidGains(0.28, 0.0, 0.0, 0.15, 0.30)
    )
    # 轴对称所以初值与俯仰同类，但一体翼偏航舵效往往略弱，先留独立一组便于微调
    yaw_ang: CascadePidGains = field(
        default_factory=lambda: CascadePidGains(3.5, 0.25, 0.04, 0.3, 1.5)
    )
    yaw_rate: CascadePidGains = field(
        default_factory=lambda: CascadePidGains(0.28, 0.0, 0.0, 0.15, 0.30)
    )


# ---------------------------------------------------------------------------
# L2 三回路（西交结构）
# ---------------------------------------------------------------------------

@dataclass
class RollThreeLoopParams:
    """滚转：外环姿态 K0、中环独立积分 K1、内环角速率阻尼 Kg。

    西交极点配置结果（他们的气动，截止约 2.5 rad/s）::
        K0=1.4776  K1=-21.636  Kg=0.5015
    符号：他们把舵机/气动符号吃进 K1。我们默认 K1>0，靠 mixer/k_act 统一符号。
    """

    # 稳定粗值：需满足 K1 < K0 (d_omega + d_delta Kg)，否则积分过大
    K0: float = 2.5
    K1: float = 3.5
    Kg: float = 0.55
    int_lim: float = 0.25
    k_act: float = 1.0  # 滚转等效舵符号；打反了只改这里或 mixer sign


@dataclass
class AccelThreeLoopParams:
    """俯仰/偏航共用：伪攻角三回路过载自动驾驶仪。

    西交结果（他们的气动，wn≈1.44 rad/s）::
        Kg=1.0213  Ka=0.1034  wi=2.4844  k_ACT=-1  K_DC=1
    尾舵 k_ACT=-1；后缘升降副翼若「正偏抬头」则取 +1。
    """

    Kg: float = 0.90
    K_alpha: float = 0.12
    wi: float = 2.2
    KA: float = 1.0
    K_DC: float = 1.0
    k_act: float = 1.0
    T_alpha: float = 0.18  # 伪攻角一阶时间常数 s
    int_lim: float = 0.6
    accel_lim_g: float = 1.2  # 外环过载指令限幅，防失速


# ---------------------------------------------------------------------------
# L3 导引
# ---------------------------------------------------------------------------

@dataclass
class GuidanceParams:
    N: float = 4.0  # 导航比，经典 3~5
    vc_fallback: float = 12.0  # 没有测接近速率时用
    los_lpf_hz: float = 8.0  # 视线角速度低通
    pixel_angle_kp: float = 1.0  # pixel_angle 模式：视线角 → 姿态指令

    # 开始制导时弹目几何（西交：下降段标定 r0, v0）
    r0: float = 18.0  # m，前哨约 16.5、基地约 25，按实际发射点改
    v0: float = 12.0  # m/s，接近速率，不是出膛速度
    r0_err: float = 2.0  # 测量上界
    v0_err: float = 2.0

    # ISMCG
    ismcg_N: float = 4.0
    ismcg_gamma: float = 8.0  # 自适应增益 1/γ
    ismcg_k_max: float = 6.0


# ---------------------------------------------------------------------------
# L1 时序 / 状态机
# ---------------------------------------------------------------------------

@dataclass
class FlightTiming:
    dt_ctrl: float = 0.005  # 200 Hz 飞控
    launch_acc_g: float = 8.0  # 过载阈值判定出发射
    t_launch_hold: float = 0.08  # 出膛保持配平
    t_coast: float = 0.35  # 只稳滚转、不导引
    t_seek_max: float = 1.20  # 超时仍无灯则保持滚转
    t_guide_min: float = 0.15  # 最短导引时间
    t_terminal: float = 0.12  # 末端减小指令，防打满舵
    t_total_max: float = 2.40
    vision_stale_s: float = 0.08  # 视觉丢失判据


# ---------------------------------------------------------------------------
# L4 耦合 / 限幅
# ---------------------------------------------------------------------------

@dataclass
class CouplingParams:
    """西交：理论三通道解耦，实测大攻角仍有滚转耦合，前馈能压住。"""

    roll_ff_alpha_beta: float = 0.0  # δr += k * α̂ * β̂ ，先 0，下降段乱滚再加
    yaw_ff_p: float = 0.0  # 逆偏航：δy += k * p
    alpha_lim_rad: float = 0.35  # ~20°，外环按伪攻角削指令
    beta_lim_rad: float = 0.35


@dataclass
class Params:
    hw: HardwareParams = field(default_factory=HardwareParams)
    aero: AeroFrozen = field(default_factory=AeroFrozen)
    cascade: CascadeParams = field(default_factory=CascadeParams)
    roll: RollThreeLoopParams = field(default_factory=RollThreeLoopParams)
    pitch: AccelThreeLoopParams = field(default_factory=AccelThreeLoopParams)
    yaw: AccelThreeLoopParams = field(default_factory=AccelThreeLoopParams)
    guid: GuidanceParams = field(default_factory=GuidanceParams)
    time: FlightTiming = field(default_factory=FlightTiming)
    couple: CouplingParams = field(default_factory=CouplingParams)

    # hold | pixel_angle | png | ismcg
    guidance_mode: str = "png"
    # cascade_pid | three_loop
    autopilot_mode: str = "cascade_pid"
    # 滚转指令始终 0（侧滑转弯 STT，相机固连弹体）
    roll_cmd_rad: float = 0.0
    gravity: float = 9.81


def default_params() -> Params:
    p = Params()
    # 轴对称 X：偏航三回路与俯仰同结构、同初值，只留独立对象方便以后微调节
    p.yaw = AccelThreeLoopParams(
        Kg=p.pitch.Kg,
        K_alpha=p.pitch.K_alpha,
        wi=p.pitch.wi,
        KA=p.pitch.KA,
        K_DC=p.pitch.K_DC,
        k_act=p.pitch.k_act,
        T_alpha=p.pitch.T_alpha,
        int_lim=p.pitch.int_lim,
        accel_lim_g=p.pitch.accel_lim_g,
    )
    return p
