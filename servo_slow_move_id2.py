# -*- coding: utf-8 -*-
"""
SCS0009 舵机慢速位置控制（ID=2）
====================================
第一版：单指缓慢移动，开环控制

方案: 软件步进控制（只写绝对目标 + 每步等待）
原因: 该舵机的速度寄存器(write_goal_speed)和定时寄存器(write_goal_time)
      在当前 rustypot 编译版下不可靠，只有位置控制可靠。
      频繁读写(<0.3s间隔)会导致舵机不响应，因此采用"写一步+等到位"的节奏。

刻度定义: 0-1000 对应 0-360°
换算公式: 刻度 u = 500 + 弧度 rad × (1000 / 2π)
          刻度 560 ↔ 21.6°，刻度 640 ↔ 50.4°

本脚本控制舵机 ID=2，活动区间限制为 560-640 刻度。

用法:
  # 移动到指定刻度（560-640 范围内，默认慢速）
  py -3.11 servo_slow_move_id2.py 600

  # 指定速度等级 (1=很慢, 2=慢, 3=中, 4=快)
  py -3.11 servo_slow_move_id2.py 600 --speed 2

  # 往复循环测试（560 <-> 640）
  py -3.11 servo_slow_move_id2.py --loop

  # 作为模块导入
  from servo_slow_move_id2 import ServoSlowMover
  mover = ServoSlowMover()
  mover.move_to(650)
"""
import time
import math
import argparse
from rustypot import Scs0009PyController

# ===== 默认配置 =====
DEFAULT_PORT = "COM3"
DEFAULT_BAUD = 1000000
DEFAULT_SID = 2            # 舵机 ID=2
MIN_USER = 560             # 刻度下限
MAX_USER = 640             # 刻度上限
TOLERANCE = 2.0            # 到位容差（刻度）

# 速度等级配置: (步长, 等待秒数)
# 整体速度 ≈ 步长 / 等待  (刻度/秒)，1 刻度 = 0.36°
SPEED_LEVELS = {
    1: (2, 0.8),    # 很慢: ~2.5 刻度/s = 0.9°/s
    2: (3, 0.6),    # 慢:   ~5.0 刻度/s = 1.8°/s  (默认)
    3: (5, 0.5),    # 中:   ~10  刻度/s = 3.6°/s
    4: (8, 0.4),    # 快:   ~20  刻度/s = 7.2°/s
}
DEFAULT_SPEED = 2


def user_to_rad(u):
    """刻度转弧度"""
    return (u - 500) * (2 * math.pi / 1000)


def rad_to_user(r):
    """弧度转刻度"""
    return 500 + r * (1000 / (2 * math.pi))


class ServoSlowMover:
    """舵机慢速移动控制器（ID=2，区间 560-640）"""

    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD, sid=DEFAULT_SID,
                 speed_level=DEFAULT_SPEED, min_user=MIN_USER, max_user=MAX_USER):
        self.port = port
        self.baud = baud
        self.sid = sid
        self.min_user = min_user
        self.max_user = max_user
        self.step, self.wait = SPEED_LEVELS[speed_level]
        self.ctrl = None

    def connect(self):
        """连接舵机"""
        self.ctrl = Scs0009PyController(
            serial_port=self.port, baudrate=self.baud, timeout=0.5
        )
        if not self.ctrl.ping(self.sid):
            raise RuntimeError(f"未检测到 ID={self.sid} 的舵机")
        self.ctrl.write_torque_enable(self.sid, 1)
        time.sleep(0.3)
        return True

    def get_position(self):
        """读取当前位置（刻度）"""
        rad = self.ctrl.read_present_position(self.sid)[0]
        return rad_to_user(rad)

    def set_speed(self, level):
        """设置速度等级 1-4"""
        if level not in SPEED_LEVELS:
            raise ValueError(f"速度等级必须是 1-4，收到 {level}")
        self.step, self.wait = SPEED_LEVELS[level]

    def move_to(self, target_user, verbose=True):
        """
        缓慢移动到目标刻度
        返回: (是否成功, 实际到达刻度)
        """
        # 范围限制
        target_user = max(self.min_user, min(self.max_user, target_user))

        if self.ctrl is None:
            self.connect()

        current = self.get_position()
        if verbose:
            print(f"  移动: {current:.1f} -> {target_user:.0f} "
                  f"(步长={self.step}, 间隔={self.wait}s, "
                  f"速度≈{self.step/self.wait:.1f}刻度/s)")

        direction = 1 if target_user > current else -1
        pos = current
        steps = 0

        # 步进移动
        while abs(target_user - pos) > self.step:
            pos += direction * self.step
            # 限制不越界
            pos = max(self.min_user, min(self.max_user, pos))
            self.ctrl.write_goal_position(self.sid, user_to_rad(pos))
            time.sleep(self.wait)
            steps += 1

        # 最后一步到位
        self.ctrl.write_goal_position(self.sid, user_to_rad(target_user))
        time.sleep(self.wait + 0.3)

        # 验证到位
        final = self.get_position()
        error = abs(final - target_user)
        success = error <= TOLERANCE

        if verbose:
            status = "OK" if success else f"偏差{error:.1f}"
            print(f"  到达: {final:.1f} (目标 {target_user:.0f}, {status}, {steps}步)")

        return success, final

    def oscillate(self, count=3, dwell=1.0):
        """在 min 和 max 之间往复移动"""
        print(f"开始往复测试: {self.min_user} <-> {self.max_user}, 共 {count} 轮")
        for i in range(count):
            print(f"\n--- 第 {i+1}/{count} 轮 ---")
            self.move_to(self.max_user)
            time.sleep(dwell)
            self.move_to(self.min_user)
            time.sleep(dwell)
        print("\n往复测试完成")


def main():
    parser = argparse.ArgumentParser(description="SCS0009 舵机慢速位置控制（ID=2）")
    parser.add_argument("target", type=float, nargs="?", default=None,
                        help=f"目标刻度 ({MIN_USER}-{MAX_USER})")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                        choices=[1, 2, 3, 4],
                        help="速度等级: 1=很慢 2=慢(默认) 3=中 4=快")
    parser.add_argument("--loop", action="store_true",
                        help=f"往复循环测试 ({MIN_USER}<->{MAX_USER})")
    parser.add_argument("--port", default=DEFAULT_PORT, help="串口号")
    parser.add_argument("--id", type=int, default=DEFAULT_SID, help="舵机ID")
    args = parser.parse_args()

    mover = ServoSlowMover(port=args.port, sid=args.id, speed_level=args.speed)

    try:
        mover.connect()
        print(f"舵机已连接: {args.port} @ {mover.baud}, ID={args.id}")
        print(f"当前位置: {mover.get_position():.1f} 刻度")
        print(f"活动区间: [{MIN_USER}, {MAX_USER}] 刻度")
        print(f"速度等级: {args.speed} (步长={mover.step}, 间隔={mover.wait}s)")

        if args.loop:
            mover.oscillate(count=3)
        elif args.target is not None:
            success, final = mover.move_to(args.target)
            if not success:
                print(f"警告: 到位偏差较大，最终位置 {final:.1f}")
        else:
            # 无参数时显示当前状态
            print("\n用法:")
            print(f"  {__file__} 600          # 移动到刻度 600")
            print(f"  {__file__} 600 --speed 1  # 很慢速度移动")
            print(f"  {__file__} --loop       # 往复测试")
    except KeyboardInterrupt:
        print("\n已停止")
    except Exception as e:
        print(f"错误: {e}")


if __name__ == "__main__":
    main()
