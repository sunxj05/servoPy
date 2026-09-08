# -*- coding: utf-8 -*-
"""
三指同步慢速移动控制
======================
第一版：三指同步缓慢移动，开环控制

方案: 软件步进同步控制（同时写三个舵机的绝对目标 + 每步等待）
效果: 三个手指从同一侧同步移动到另一侧，保持进度一致

舵机配置:
  ID=1: 区间 [610, 700] 刻度（起点 610，下限，方向正常）
  ID=2: 区间 [580, 630] 刻度（方向反转）
  ID=4: 区间 [630, 720] 刻度（方向反转）

同步逻辑: 用进度 0.0~1.0 表示在各自区间内的相对位置
  ID=1（正常）: 进度 0 = 下限 610，进度 1 = 上限 700
  ID=2（反转）: 进度 0 = 上限 630，进度 1 = 下限 580
  ID=4（反转）: 进度 0 = 上限 720，进度 1 = 下限 630
  每步同时给三个舵机下发对应进度的目标刻度，实现物理方向同步

运动方向:
  下限 -> 上限: 慢速步进同步（受控），ID=1 以 610 作为起点
  上限 -> 下限: 中速回弹（步进控制，比上行快）
  每次到达上限后自动回弹到下限，可循环多次

用法:
  # 三指慢速从下限同步到上限，然后中速回弹到下限（1次）
  py -3.11 three_finger_sync_move.py --sweep-up

  # 循环3次（每次到达上限后自动回弹）
  py -3.11 three_finger_sync_move.py --sweep-up --count 3

  # 三指从当前位置同步移动到指定进度
  py -3.11 three_finger_sync_move.py 0.5

  # 指定上行速度等级 (1=很慢, 2=慢, 3=中, 4=快)
  py -3.11 three_finger_sync_move.py --sweep-up --speed 2
"""
import time
import math
import argparse
from rustypot import Scs0009PyController

# ===== 默认配置 =====
DEFAULT_PORT = "COM3"
DEFAULT_BAUD = 1000000

# 三指配置: (舵机ID, 刻度下限, 刻度上限, 方向反转)
# reverse=True 时，进度 0=上限，进度 1=下限（物理方向与正常舵机相反）
FINGER_1 = {"id": 1, "min": 610, "max": 640, "reverse": False}  # ID=1，方向正常
FINGER_2 = {"id": 2, "min": 600, "max": 640, "reverse": True}   # ID=2，方向反转
FINGER_4 = {"id": 4, "min": 650, "max": 720, "reverse": True}   # ID=4，方向反转

FINGERS = [FINGER_1, FINGER_2, FINGER_4]

TOLERANCE = 2.0      # 到位容差（刻度）

# 速度等级配置: (步长, 等待秒数)
# 步长单位是进度(0-1)，整体速度 ≈ 步长/等待
SPEED_LEVELS = {
    1: (0.02, 0.8),   # 很慢
    2: (0.03, 0.6),   # 慢 (默认)
    3: (0.05, 0.5),   # 中
    4: (0.08, 0.4),   # 快
}
DEFAULT_SPEED = 2

# 中速回弹参数
MEDIUM_STEP = 0.06
MEDIUM_WAIT = 0.35


def progress_to_tick(finger, progress):
    """进度(0-1)转舵机刻度，支持方向反转"""
    if finger.get("reverse", False):
        # 反转: 进度 0=上限，进度 1=下限
        return finger["max"] - (finger["max"] - finger["min"]) * progress
    else:
        # 正常: 进度 0=下限，进度 1=上限
        return finger["min"] + (finger["max"] - finger["min"]) * progress


def tick_to_progress(finger, tick):
    """刻度转进度(0-1)，支持方向反转"""
    if finger.get("reverse", False):
        return (finger["max"] - tick) / (finger["max"] - finger["min"])
    else:
        return (tick - finger["min"]) / (finger["max"] - finger["min"])


def user_to_rad(u):
    """刻度转弧度"""
    return (u - 500) * (2 * math.pi / 1000)


def rad_to_user(r):
    """弧度转刻度"""
    return 500 + r * (1000 / (2 * math.pi))


class ThreeFingerSyncMover:
    """三指同步慢速移动控制器"""

    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD,
                 fingers=FINGERS, speed_level=DEFAULT_SPEED):
        self.port = port
        self.baud = baud
        self.fingers = fingers
        self.step, self.wait = SPEED_LEVELS[speed_level]
        self.ctrl = None

    def connect(self):
        """连接三个舵机"""
        self.ctrl = Scs0009PyController(
            serial_port=self.port, baudrate=self.baud, timeout=0.5
        )
        for f in self.fingers:
            if not self.ctrl.ping(f["id"]):
                raise RuntimeError(f"未检测到 ID={f['id']} 的舵机")
            self.ctrl.write_torque_enable(f["id"], 1)
            time.sleep(0.15)
        time.sleep(0.2)
        return True

    def get_positions(self):
        """读取三个舵机的当前位置（刻度和进度）"""
        result = {}
        for f in self.fingers:
            tick = rad_to_user(self.ctrl.read_present_position(f["id"])[0])
            progress = tick_to_progress(f, tick)
            result[f["id"]] = {"tick": tick, "progress": progress}
        return result

    def set_speed(self, level):
        """设置速度等级 1-4"""
        if level not in SPEED_LEVELS:
            raise ValueError(f"速度等级必须是 1-4，收到 {level}")
        self.step, self.wait = SPEED_LEVELS[level]

    def move_to_progress(self, target_progress, verbose=True):
        """
        三指同步移动到指定进度(0-1)，慢速步进
        返回: (是否全部到位, 各舵机最终位置)
        """
        target_progress = max(0.0, min(1.0, target_progress))

        if self.ctrl is None:
            self.connect()

        pos = self.get_positions()
        # 以三个手指的平均进度作为当前同步进度
        cur_progress = sum(pos[f["id"]]["progress"] for f in self.fingers) / len(self.fingers)

        if verbose:
            print(f"  同步移动: 进度 {cur_progress:.2f} -> {target_progress:.2f}")
            for f in self.fingers:
                t = progress_to_tick(f, target_progress)
                print(f"    ID={f['id']}: {pos[f['id']]['tick']:.1f} -> {t:.1f} 刻度")
            print(f"    (步长={self.step}, 间隔={self.wait}s)")

        direction = 1 if target_progress > cur_progress else -1
        p = cur_progress
        steps = 0

        # 步进同步移动
        while abs(target_progress - p) > self.step:
            p += direction * self.step
            p = max(0.0, min(1.0, p))
            # 同时下发三个舵机的目标位置
            for f in self.fingers:
                self.ctrl.write_goal_position(
                    f["id"], user_to_rad(progress_to_tick(f, p)))
            time.sleep(self.wait)
            steps += 1

        # 最后一步到位
        for f in self.fingers:
            self.ctrl.write_goal_position(
                f["id"], user_to_rad(progress_to_tick(f, target_progress)))
        time.sleep(self.wait + 0.3)

        # 验证到位
        final = self.get_positions()
        success = True
        for f in self.fingers:
            err = abs(final[f["id"]]["tick"] - progress_to_tick(f, target_progress))
            if err > TOLERANCE:
                success = False

        if verbose:
            parts = [f"ID={f['id']} {final[f['id']]['tick']:.1f}" for f in self.fingers]
            print(f"  到达: {', '.join(parts)} ({steps}步)")

        return success, final

    def move_medium_to_progress(self, target_progress, verbose=True):
        """
        三指中速移动到指定进度（步进控制，比慢速快）
        用于回弹方向
        """
        target_progress = max(0.0, min(1.0, target_progress))

        if self.ctrl is None:
            self.connect()

        pos = self.get_positions()
        cur_progress = sum(pos[f["id"]]["progress"] for f in self.fingers) / len(self.fingers)

        if verbose:
            print(f"  中速移动: 进度 {cur_progress:.2f} -> {target_progress:.2f}")
            for f in self.fingers:
                t = progress_to_tick(f, target_progress)
                print(f"    ID={f['id']}: {pos[f['id']]['tick']:.1f} -> {t:.1f}")

        direction = 1 if target_progress > cur_progress else -1
        p = cur_progress
        steps = 0

        while abs(target_progress - p) > MEDIUM_STEP:
            p += direction * MEDIUM_STEP
            p = max(0.0, min(1.0, p))
            for f in self.fingers:
                self.ctrl.write_goal_position(
                    f["id"], user_to_rad(progress_to_tick(f, p)))
            time.sleep(MEDIUM_WAIT)
            steps += 1

        # 最后一步到位
        for f in self.fingers:
            self.ctrl.write_goal_position(
                f["id"], user_to_rad(progress_to_tick(f, target_progress)))
        time.sleep(MEDIUM_WAIT + 0.2)

        final = self.get_positions()
        if verbose:
            parts = [f"ID={f['id']} {final[f['id']]['tick']:.1f}" for f in self.fingers]
            print(f"  到位: {', '.join(parts)} ({steps}步)")

        return final

    def sweep_up_medium_back(self, count=1, verbose=True):
        """
        三指慢速从下限同步到上限，然后中速回弹到下限
        慢速方向: 下限(进度0.0) -> 上限(进度1.0)，ID=1 以 610 作为起点
        中速回弹: 上限(进度1.0) -> 下限(进度0.0)
        每次到达上限后自动回弹，循环 count 次
        """
        if verbose:
            print(f"=== 三指循环动作: 慢速上行(下限->上限) + 中速回弹(上限->下限)，共 {count} 轮 ===")

        # 第一步: 先中速到下限起始位置（ID=1 到 610）
        if verbose:
            print("\n[初始化] 中速回到下限起始位 (ID=1 -> 610)")
        self.move_medium_to_progress(0.0, verbose=verbose)
        time.sleep(0.5)

        success = True
        for i in range(count):
            if verbose:
                print(f"\n--- 第 {i+1}/{count} 轮 ---")

            # 第二步: 慢速从下限同步到上限
            if verbose:
                print(f"  [上行] 慢速同步 (下限 -> 上限)")
            ok, final = self.move_to_progress(1.0, verbose=verbose)
            success = success and ok
            if verbose:
                print(f"  [延时] 上行结束，等待 3 秒后回弹...")
            time.sleep(3.0)

            # 第三步: 中速回弹到下限（到达上限后自动回弹）
            if verbose:
                print(f"  [回弹] 中速回弹 (上限 -> 下限)")
            self.move_medium_to_progress(0.0, verbose=verbose)
            time.sleep(0.3)

        if verbose:
            print(f"\n=== {count} 轮动作完成 ===")
        return success


def main():
    parser = argparse.ArgumentParser(description="三指同步慢速位置控制")
    parser.add_argument("target", type=float, nargs="?", default=None,
                        help="目标进度 (0.0-1.0)，0=各自下限，1=各自上限")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                        choices=[1, 2, 3, 4],
                        help="上行速度等级: 1=很慢 2=慢(默认) 3=中 4=快")
    parser.add_argument("--sweep-up", action="store_true",
                        help="三指慢速上行(下限->上限) + 中速回弹(上限->下限)")
    parser.add_argument("--count", type=int, default=1,
                        help="循环次数（默认1次，每次到达上限后自动回弹）")
    parser.add_argument("--port", default=DEFAULT_PORT, help="串口号")
    args = parser.parse_args()

    mover = ThreeFingerSyncMover(port=args.port, speed_level=args.speed)

    try:
        mover.connect()
        print(f"三指已连接: {args.port} @ {mover.baud}")
        pos = mover.get_positions()
        parts = [f"ID={f['id']} {pos[f['id']]['tick']:.1f}(进度{pos[f['id']]['progress']:.2f})"
                 for f in mover.fingers]
        print(f"当前位置: {', '.join(parts)}")
        print(f"上行速度等级: {args.speed} (步长={mover.step}, 间隔={mover.wait}s)")

        if args.sweep_up:
            mover.sweep_up_medium_back(count=args.count)
        elif args.target is not None:
            success, final = mover.move_to_progress(args.target)
            if not success:
                print("警告: 部分舵机到位偏差较大")
        else:
            print("\n用法:")
            print(f"  {__file__} --sweep-up           # 慢速上行+中速回弹(1次)")
            print(f"  {__file__} --sweep-up --count 3 # 循环3次")
            print(f"  {__file__} 0.5                    # 三指同步移动到指定进度")
    except KeyboardInterrupt:
        print("\n已停止")
    except Exception as e:
        print(f"错误: {e}")


if __name__ == "__main__":
    main()
