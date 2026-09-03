# -*- coding: utf-8 -*-
"""
SCS0009 舵机抓取检测 —— 实时偏移量判定是否抓住物品
======================================================
思路:
  让手指向内收(抓握方向)逐步移动，每步之后读回实际位置，
  计算"偏移量 = 指令刻度 - 实际刻度"(按收拢方向取正)。
    - 没碰到东西: 实际位置紧跟指令，偏移量接近 0
    - 顶到物品:   实际位置被卡住，偏移量不断增大
    - 偏移量 >= 阈值(默认 10 刻度) → 判定"已抓住"，该指停止
  全程遵循各手指的刻度限制区间，不越界。

原理与之前相同: 速度/定时寄存器在当前 rustypot 版下不可靠，
因此仍用"软件步进"——只写绝对目标 + 每步等待 ≥0.4s，写一步读一步。

刻度定义: 0-1000 对应 0-360°，1 刻度 = 0.36°

用法示例:
  # 三指同时内收抓取，偏移量达 10 即停止
  py -3.11 servo_grasp_detect.py grasp

  # 自定义阈值 / 只抓指定手指 / 收拢方向朝下限
  py -3.11 servo_grasp_detect.py grasp --threshold 8 --fingers 2 4 --close min

  # 松开(回到张开限位)
  py -3.11 servo_grasp_detect.py open

  # 作为模块导入
  from servo_grasp_detect import GraspDetector
  det = GraspDetector()
  det.connect([1, 2, 4])
  det.grasp([1, 2, 4])
"""
import time
import math
import argparse
from rustypot import Scs0009PyController

# ===== 默认配置 =====
DEFAULT_PORT = "COM3"
DEFAULT_BAUD = 1000000

# 手指配置: 舵机ID -> (刻度下限, 刻度上限)
# 已按实测在线舵机(ID=1/2/4)修正；ID=1/2 区间暂定 600-700，ID=4 为 600-750，
# 若某根手指的机械限位不同请在此调整。
FINGER_LIMITS = {
    1: (600, 700),
    2: (600, 700),
    4: (600, 750),
}

THRESHOLD = 10.0      # 抓取判定偏移量阈值（刻度）
TOLERANCE = 2.0       # 到位容差（刻度）

# 速度等级配置: (步长, 等待秒数)
SPEED_LEVELS = {
    1: (2, 0.8),    # 很慢
    2: (3, 0.6),    # 慢（默认）
    3: (5, 0.5),    # 中
    4: (8, 0.4),    # 快
}
DEFAULT_SPEED = 2


def user_to_rad(u):
    """刻度转弧度"""
    return (u - 500) * (2 * math.pi / 1000)


def rad_to_user(r):
    """弧度转刻度"""
    return 500 + r * (1000 / (2 * math.pi))


class GraspDetector:
    """基于实时偏移量的抓取检测控制器（支持多指）"""

    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD,
                 speed_level=DEFAULT_SPEED, threshold=THRESHOLD,
                 close_dir="max"):
        self.port = port
        self.baud = baud
        self.step, self.wait = SPEED_LEVELS[speed_level]
        self.threshold = threshold
        self.close_dir = close_dir      # "max"=朝上限收拢, "min"=朝下限收拢
        self.ctrl = None

    # ---------- 连接 ----------
    def connect(self, sids):
        """连接串口并确认哪些手指在线，返回在线ID列表"""
        self.ctrl = Scs0009PyController(
            serial_port=self.port, baudrate=self.baud, timeout=0.5
        )
        online = [s for s in sids if self.ctrl.ping(s)]
        if not online:
            raise RuntimeError(f"未检测到任何在线手指舵机 {sids}")
        for s in online:
            self.ctrl.write_torque_enable(s, 1)
        time.sleep(0.3)
        return online

    def get_position(self, sid):
        """读取当前实际位置（刻度）"""
        rad = self.ctrl.read_present_position(sid)[0]
        return rad_to_user(rad)

    def _limits(self, sid):
        return FINGER_LIMITS.get(sid, (600, 700))

    def _closing_limit(self, sid):
        """内收（抓握）方向的限位刻度"""
        lo, hi = self._limits(sid)
        return hi if self.close_dir == "max" else lo

    def _open_limit(self, sid):
        """张开方向的限位刻度"""
        lo, hi = self._limits(sid)
        return lo if self.close_dir == "max" else hi

    # ---------- 抓取 ----------
    def grasp(self, sids, verbose=True):
        """
        多指同时向内收，实时检测偏移量。
        每根手指: 指令向前推一步 -> 等待 -> 读回实际位置 ->
                  偏移量 >= 阈值 -> 判为"已抓住"并停止该指。
        所有手指都"已抓住"或"到限位"后结束。
        返回: {sid: {status, cmd, cur, grasp_at}}
        """
        state = {}
        for sid in sids:
            cur = self.get_position(sid)
            lim = self._closing_limit(sid)
            direction = 1 if lim > cur else -1
            state[sid] = {
                "cmd": cur, direction: direction, "limit": lim,
                "status": "moving", "grasp_at": None,
            }

        if verbose:
            print(f"抓取开始: 手指={sids}, 收拢方向={self.close_dir}, "
                  f"偏移阈值={self.threshold}刻度")
            print(f"  速度: 步长={self.step}, 间隔={self.wait}s")

        step_no = 0
        while True:
            moving = [s for s, st in state.items() if st["status"] == "moving"]
            if not moving:
                break

            # 1) 对仍在移动的手指各下发一步（不越过收拢限位）
            for sid in moving:
                st = state[sid]
                nxt = st["cmd"] + st["direction"] * self.step
                if st["direction"] > 0:
                    nxt = min(nxt, st["limit"])
                else:
                    nxt = max(nxt, st["limit"])
                st["cmd"] = nxt
                self.ctrl.write_goal_position(sid, user_to_rad(nxt))
            time.sleep(self.wait)
            step_no += 1

            # 2) 读回实际位置，计算偏移量判定
            for sid in moving:
                st = state[sid]
                actual = self.get_position(sid)
                st["cur"] = actual
                offset = (st["cmd"] - actual) * st["direction"]  # 收拢方向为正
                if offset >= self.threshold:
                    st["status"] = "grasped"      # 顶到东西，抓住
                    st["grasp_at"] = actual
                elif st["cmd"] == st["limit"]:     # 指令已到限位且无接触
                    st["status"] = "at_limit"
                elif abs(actual - st["limit"]) <= TOLERANCE:
                    st["status"] = "at_limit"      # 实际已到限位

            if verbose:
                parts = []
                for sid, st in state.items():
                    off = abs((st["cmd"] - st["cur"]) * st["direction"])
                    parts.append(f"ID{sid}: 指{st['cmd']:.1f}/实{st['cur']:.1f}/"
                                 f"偏{off:.1f}/{st['status']}")
                print(f"  步{step_no}: " + " | ".join(parts))

        # 3) 汇总结果
        print("\n抓取结果:")
        for sid, st in state.items():
            if st["status"] == "grasped":
                print(f"  ID{sid}: 已抓住物品 (停止在 {st['grasp_at']:.1f} 刻度)")
            else:
                print(f"  ID{sid}: 未接触物品，到限位 {st['limit']:.0f} 刻度")
        return state

    # ---------- 松开 ----------
    def open_hand(self, sids, verbose=True):
        """所有手指缓慢回到张开限位（释放物品）"""
        if verbose:
            print(f"松开: 手指={sids}")
        for sid in sids:
            cur = self.get_position(sid)
            target = self._open_limit(sid)
            direction = 1 if target > cur else -1
            pos = cur
            steps = 0
            while abs(target - pos) > self.step:
                pos += direction * self.step
                if direction > 0:
                    pos = min(pos, target)
                else:
                    pos = max(pos, target)
                self.ctrl.write_goal_position(sid, user_to_rad(pos))
                time.sleep(self.wait)
                steps += 1
            self.ctrl.write_goal_position(sid, user_to_rad(target))
            time.sleep(self.wait + 0.3)
            final = self.get_position(sid)
            if verbose:
                print(f"  ID{sid}: {cur:.1f} -> 目标{target:.0f}, 实际{final:.1f}, {steps}步")


def main():
    parser = argparse.ArgumentParser(
        description="SCS0009 舵机抓取检测（实时偏移量判定是否抓住物品）")
    parser.add_argument("mode", nargs="?", default="grasp",
                        choices=["grasp", "open"],
                        help="grasp=内收抓取(默认), open=松开")
    parser.add_argument("--fingers", type=int, nargs="+", default=None,
                        help=f"参与的手指舵机ID，默认用配置里全部 {list(FINGER_LIMITS.keys())}")
    parser.add_argument("--threshold", type=float, default=THRESHOLD,
                        help=f"抓取判定偏移量阈值(刻度)，默认 {THRESHOLD}")
    parser.add_argument("--close", choices=["max", "min"], default="max",
                        help="内收方向: 朝刻度上限(max,默认)或下限(min)")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                        choices=[1, 2, 3, 4],
                        help="速度等级: 1=很慢 2=慢(默认) 3=中 4=快")
    parser.add_argument("--port", default=DEFAULT_PORT, help="串口号")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="波特率")
    args = parser.parse_args()

    det = GraspDetector(port=args.port, baud=args.baud,
                        speed_level=args.speed,
                        threshold=args.threshold, close_dir=args.close)
    fingers = args.fingers or list(FINGER_LIMITS.keys())

    try:
        online = det.connect(fingers)
        offline = [f for f in fingers if f not in online]
        if offline:
            print(f"警告: 以下手指不在线，已跳过 {offline}")
        print(f"已连接: {args.port} @ {det.baud}, 手指={online}")
        for s in online:
            lo, hi = det._limits(s)
            print(f"  ID{s}: 当前 {det.get_position(s):.1f} 刻度, 区间 [{lo},{hi}]")

        if args.mode == "grasp":
            det.grasp(online)
        else:
            det.open_hand(online)
    except KeyboardInterrupt:
        print("\n已停止（手指保持当前力矩，未继续下发）")
    except Exception as e:
        print(f"错误: {e}")


if __name__ == "__main__":
    main()
