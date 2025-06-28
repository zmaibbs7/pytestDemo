# project/core/motion.py
"""
运动规划模块 - 基于距离和角度的精确控制
职责：
1. 提供基于距离的移动接口
2. 提供基于角度的转向接口 
3. 统一的执行频率控制
4. 运动状态监控
"""

import time
import math
import threading
from typing import Optional
from dataclasses import dataclass

from .controller import RobotController


@dataclass
class MotionConfig:
    """运动配置"""
    update_rate: float = 20.0  # 控制频率 Hz
    position_tolerance: float = 0.05  # 位置容差 m
    angle_tolerance: float = 0.05  # 角度容差 rad
    
    @property
    def update_interval(self) -> float:
        """更新间隔"""
        return 1.0 / self.update_rate

class MotionPlanner:
    """
    运动规划器 - 精确的距离和角度控制
    
    使用方式:
    planner.move_forward(0.2, 0.5)  # 前进0.2米，速度0.5
    planner.turn_left(90)           # 左转90度
    planner.move_forward(0.4, 0.3)  # 再前进0.4米，速度0.3
    """
    
    def __init__(self, controller: RobotController, config: Optional[MotionConfig] = None):
        """
        初始化运动规划器
        
        Args:
            controller: 机器人控制器
            config: 运动配置
        """
        self.controller = controller
        self.config = config or MotionConfig()
        
        # 执行控制
        self._stop_flag = threading.Event()
        self._is_executing = False
        self._current_thread = None

    def set_update_rate(self, rate: float):
        """设置控制频率"""
        self.config.update_rate = rate

    def _execute_timed_motion(self, linear_speed: float, angular_speed: float, duration: float) -> bool:
        """执行定时运动"""
        self._is_executing = True
        self._stop_flag.clear()
        
        try:
            start_time = time.time()
            end_time = start_time + duration
            
            while time.time() < end_time and not self._stop_flag.is_set():
                # 发送速度命令
                success = self.controller.send_velocity_command(linear_speed, angular_speed)
                if not success:
                    return False
                    
                # 等待下一个控制周期
                time.sleep(self.config.update_interval)
            
            # 运动结束，停止
            self.controller.stop()
            return True
            
        except Exception as e:
            print(f"运动执行异常: {e}")
            self.controller.emergency_stop()
            return False
        finally:
            self._is_executing = False

    def move_forward(self, distance: float, speed: float = 0.5) -> bool:
        """
        向前移动指定距离
        
        Args:
            distance: 移动距离 (米)
            speed: 移动速度 (0-1.0)
            
        Returns:
            bool: 是否执行成功
        """
        if distance <= 0:
            print("移动距离必须大于0")
            return False
            
        if not 0 < speed <= 1.0:
            print("速度必须在 0-1.0 范围内")
            return False
        
        # 计算实际速度和执行时间
        actual_speed = speed * self.controller.max_linear_speed
        duration = distance / actual_speed
        
        print(f"🟢 前进 {distance}m，速度 {speed}，预计用时 {duration:.2f}s")
        
        return self._execute_timed_motion(speed, 0.0, duration)

    def move_backward(self, distance: float, speed: float = 0.5) -> bool:
        """
        向后移动指定距离
        
        Args:
            distance: 移动距离 (米)
            speed: 移动速度 (0-1.0)
            
        Returns:
            bool: 是否执行成功
        """
        if distance <= 0:
            print("移动距离必须大于0")
            return False
            
        if not 0 < speed <= 1.0:
            print("速度必须在 0-1.0 范围内")
            return False
        
        # 计算实际速度和执行时间
        actual_speed = speed * self.controller.max_linear_speed
        duration = distance / actual_speed
        
        print(f"🔴 后退 {distance}m，速度 {speed}，预计用时 {duration:.2f}s")
        
        return self._execute_timed_motion(-speed, 0.0, duration)

    def turn_left(self, angle_degrees: float, speed: float = 0.5) -> bool:
        """
        左转指定角度
        
        Args:
            angle_degrees: 转向角度 (度)
            speed: 转向速度 (0-1.0)
            
        Returns:
            bool: 是否执行成功
        """
        if angle_degrees <= 0:
            print("转向角度必须大于0")
            return False
            
        if not 0 < speed <= 1.0:
            print("速度必须在 0-1.0 范围内")
            return False
        
        # 转换为弧度
        angle_rad = math.radians(angle_degrees)
        
        # 计算实际角速度和执行时间
        actual_angular_speed = speed * self.controller.max_angular_speed
        duration = angle_rad / actual_angular_speed
        
        print(f"↰ 左转 {angle_degrees}°，速度 {speed}，预计用时 {duration:.2f}s")
        
        return self._execute_timed_motion(0.0, speed, duration)

    def turn_right(self, angle_degrees: float, speed: float = 0.5) -> bool:
        """
        右转指定角度
        
        Args:
            angle_degrees: 转向角度 (度)
            speed: 转向速度 (0-1.0)
            
        Returns:
            bool: 是否执行成功
        """
        if angle_degrees <= 0:
            print("转向角度必须大于0")
            return False
            
        if not 0 < speed <= 1.0:
            print("速度必须在 0-1.0 范围内")
            return False
        
        # 转换为弧度
        angle_rad = math.radians(angle_degrees)
        
        # 计算实际角速度和执行时间
        actual_angular_speed = speed * self.controller.max_angular_speed
        duration = angle_rad / actual_angular_speed
        
        print(f"↱ 右转 {angle_degrees}°，速度 {speed}，预计用时 {duration:.2f}s")
        
        return self._execute_timed_motion(0.0, -speed, duration)

    def stop(self):
        """停止当前运动"""
        self._stop_flag.set()
        self.controller.stop()
        
        if self._current_thread and self._current_thread.is_alive():
            self._current_thread.join(timeout=1.0)
        
        print("⏹️ 运动已停止")

    def emergency_stop(self):
        """紧急停止"""
        self._stop_flag.set()
        self.controller.emergency_stop()
        print("🚨 紧急停止")

    def is_executing(self) -> bool:
        """是否正在执行运动"""
        return self._is_executing

    # 运动形状方法
    def move_square(self, side_length: float, speed: float = 0.3) -> bool:
        """
        走正方形
        
        Args:
            side_length: 边长 (米)
            speed: 速度 (0-1.0)
        """
        print(f"🟦 开始走正方形，边长 {side_length}m")
        
        try:
            for i in range(4):
                if self._stop_flag.is_set():
                    break
                    
                print(f"  📍 第{i+1}条边")
                if not self.move_forward(side_length, speed):
                    return False
                    
                if not self.turn_left(90, speed):
                    return False
                    
            print("✅ 正方形完成")
            return True
            
        except Exception as e:
            print(f"正方形运动异常: {e}")
            self.emergency_stop()
            return False

    def move_rectangle(self, length: float, width: float, speed: float = 0.3) -> bool:
        """
        走长方形
        
        Args:
            length: 长边长度 (米)
            width: 短边长度 (米)
            speed: 速度 (0-1.0)
        """
        print(f"🟨 开始走长方形，长 {length}m，宽 {width}m")
        
        try:
            sides = [length, width, length, width]
            
            for i, side in enumerate(sides):
                if self._stop_flag.is_set():
                    break
                    
                print(f"  📍 第{i+1}条边 ({side}m)")
                if not self.move_forward(side, speed):
                    return False
                    
                if not self.turn_left(90, speed):
                    return False
                    
            print("✅ 长方形完成")
            return True
            
        except Exception as e:
            print(f"长方形运动异常: {e}")
            self.emergency_stop()
            return False

    def move_triangle(self, side_length: float, speed: float = 0.3) -> bool:
        """
        走三角形 (等边三角形)
        
        Args:
            side_length: 边长 (米)
            speed: 速度 (0-1.0)
        """
        print(f"🔺 开始走三角形，边长 {side_length}m")
        
        try:
            for i in range(3):
                if self._stop_flag.is_set():
                    break
                    
                print(f"  📍 第{i+1}条边")
                if not self.move_forward(side_length, speed):
                    return False
                    
                # 等边三角形外角为120度
                if not self.turn_left(120, speed):
                    return False
                    
            print("✅ 三角形完成")
            return True
            
        except Exception as e:
            print(f"三角形运动异常: {e}")
            self.emergency_stop()
            return False

    def move_circle(self, radius: float, speed: float = 0.3, segments: int = 36) -> bool:
        """
        走圆形（多段折线近似）

        Args:
            radius   : 圆半径 (米)
            speed    : 线速度比例 (0-1.0)
            segments : 分段数量 (>=4)。越大越平滑，默认 36 段≈10°/段
        """
        print(f"🟠 开始走圆形，半径 {radius}m，分 {segments} 段，速度 {speed}")

        # ---------- 参数校验 ----------
        if radius <= 0:
            print("半径必须大于 0")
            return False
        if not 0 < speed <= 1.0:
            print("速度必须在 0-1.0 范围内")
            return False
        if segments < 4:
            print("分段数量至少为 4")
            return False

        # ---------- 计算每段长度与转角 ----------
        circumference = 2 * math.pi * radius
        segment_len   = circumference / segments          # 每段近似直线长度
        turn_angle    = 360.0 / segments                  # 每段外角 (度)

        try:
            for i in range(segments):
                if self._stop_flag.is_set():
                    break

                print(f"  📍 第{i+1}段 ({segment_len:.3f}m)")
                if not self.move_forward(segment_len, speed):
                    return False

                # 画圆时始终左转
                if not self.turn_left(turn_angle, speed):
                    return False

            print("✅ 圆形完成")
            return True

        except Exception as e:
            print(f"圆形运动异常: {e}")
            self.emergency_stop()
            return False

    # 形状中心移动
    def move_to_center(self, shape: str, **dims) -> bool:
        """
        根据已知尺寸，将机器人从当前“起点”移动到图形几何中心附近。
        
        假设：画完图形后，机器人回到起点且朝向与开始时一致。
        
        Args:
            shape: 'square' | 'rectangle' | 'triangle' | 'circle'
            dims : 对应形状的尺寸参数  
                   - square     ➜ side_length=float  
                   - rectangle  ➜ length=float, width=float  
                   - triangle   ➜ side_length=float  (等边)  
                   - circle     ➜ radius=float
        
        Returns:
            bool: 动作是否全部成功
        """
        shape = shape.lower()
        try:
            if shape == 'square':
                s = dims.get('side_length')
                if s is None:
                    raise ValueError("square 需要 side_length")
                return (
                    self.move_forward(s / 2) and
                    self.turn_left(90) and
                    self.move_forward(s / 2)
                )

            elif shape == 'rectangle':
                l = dims.get('length')
                w = dims.get('width')
                if l is None or w is None:
                    raise ValueError("rectangle 需要 length 与 width")
                return (
                    self.move_forward(l / 2) and
                    self.turn_left(90) and
                    self.move_forward(w / 2)
                )

            elif shape == 'triangle':
                a = dims.get('side_length')
                if a is None:
                    raise ValueError("triangle 需要 side_length")
                # 顶点到重心距离 = √3 / 3 * a
                dist = (math.sqrt(3) / 3) * a
                return (
                    self.turn_left(30) and
                    self.move_forward(dist)
                )

            elif shape == 'circle':
                r = dims.get('radius')
                if r is None:
                    raise ValueError("circle 需要 radius")
                return (
                    self.turn_left(90) and
                    self.move_forward(r)
                )

            else:
                raise ValueError(f"暂不支持的形状类型: {shape}")

        except Exception as e:
            print(f"移动到中心异常: {e}")
            self.emergency_stop()
            return False