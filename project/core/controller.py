# project/core/controller.py
"""
机器人控制器核心模块 - 领域层，机型无关
职责：
1. 基础运动控制抽象
2. 速度标准化和安全限制
3. 控制指令的标准化格式
"""

import time
import struct
from typing import Protocol, Optional, Callable
from dataclasses import dataclass
from enum import Enum

from project.infrastructure.ros.pub import publish_ble


@dataclass
class VelocityCommand:
    """速度控制命令数据结构"""
    linear_velocity: float  # 标准化线速度 (-1.0 ~ 1.0)
    angular_velocity: float  # 标准化角速度 (-1.0 ~ 1.0)
    control_flags: int = 0  # 控制标志位
    timestamp: float = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()

class CommandPublisher(Protocol):
    """命令发布接口 - 由基础设施层实现"""
    def publish_command(self, payload: str) -> None:
        ...
    
class DefaultCommandPublisher:
    """
    默认命令发布器：使用 ROS pub.py 中的 publish_command 函数
    """
    def __init__(self, topic: str = "/ble/cmd"):
        self.topic = topic

    def publish_command(self, payload: str) -> None:
        success = publish_ble(self.topic, payload)
        if not success:
            raise RuntimeError(f"[DefaultCommandPublisher] 命令发布失败: topic={self.topic}, payload={payload}")

class RobotController:
    """
    机器人底层控制器 - 核心控制逻辑
    
    职责：
    - 速度指令的标准化和验证
    - 控制数据格式转换
    - 安全限制检查
    """
    
    def __init__(self, 
                 max_linear_speed: float = 0.45,
                 max_angular_speed: float = 0.36,
                 publisher: CommandPublisher = DefaultCommandPublisher(),
                 debug_callback: Optional[Callable[[str], None]] = None):
        """
        初始化控制器
        
        Args:
            max_linear_speed: 最大线速度 (m/s)
            max_angular_speed: 最大角速度 (rad/s)
            publisher: 命令发布器
            debug_callback: 调试回调函数
        """
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.publisher = publisher
        self.debug_callback = debug_callback or self._default_debug
        
        self._last_command: Optional[VelocityCommand] = None

    def _default_debug(self, message: str):
        """默认调试输出"""
        print(f"[Controller] {message}")

    def _check_velocity(self, linear_vel: float, angular_vel: float) -> tuple[float, float]:
        """检查和限制速度值"""
        # 限制到 [-1.0, 1.0] 范围
        linear_vel = max(-1.0, min(1.0, linear_vel))
        angular_vel = max(-1.0, min(1.0, angular_vel))
        
        return linear_vel, angular_vel

    def _scale_speed(self, linear_vel: float, angular_vel: float) -> tuple[int, int]:
        """将标准化速度转换为内部表示"""
        # 转换为实际速度值并缩放到int16范围
        linear_scaled = round(linear_vel * self.max_linear_speed * 100)
        angular_scaled = round(angular_vel * (-self.max_angular_speed) * 100)  # 角速度反转
        
        return linear_scaled, angular_scaled

    def _int16_to_bytes(self, value: int) -> list[int]:
        """将int16转换为字节数组 (小端序)"""
        value = max(-32768, min(32767, value))  # 限制到int16范围
        bytes_data = struct.pack('<h', value)
        return list(bytes_data)

    def _create_payload(self, command: VelocityCommand) -> str:
        """创建控制命令载荷"""
        # 检查速度
        linear_vel, angular_vel = self._check_velocity(
            command.linear_velocity, command.angular_velocity
        )
        
        # 缩放速度
        linear_scaled, angular_scaled = self._scale_speed(linear_vel, angular_vel)
        
        # 转换为字节数组
        linear_bytes = self._int16_to_bytes(linear_scaled)
        angular_bytes = self._int16_to_bytes(angular_scaled)
        
        # 组合数据包
        data = linear_bytes + angular_bytes + [command.control_flags]
        data_unsigned = [b & 0xFF for b in data]  # 转换为无符号显示
        
        payload = f'{{"d":{data_unsigned},"m":"r"}}'
        return payload

    def send_velocity_command(self, 
                            linear_velocity: float, 
                            angular_velocity: float,
                            control_flags: int = 0) -> bool:
        """
        发送速度控制指令
        
        Args:
            linear_velocity: 标准化线速度 (-1.0 ~ 1.0)
            angular_velocity: 标准化角速度 (-1.0 ~ 1.0)
            control_flags: 控制标志位
            
        Returns:
            bool: 是否发送成功
        """
        try:
            # 创建命令对象
            command = VelocityCommand(
                linear_velocity=linear_velocity,
                angular_velocity=angular_velocity,
                control_flags=control_flags
            )
            
            # 生成载荷
            payload = self._create_payload(command)
            
            # 发布命令
            if self.publisher:
                self.publisher.publish_command(payload)
            
            # 记录命令
            self._last_command = command
            
            # 调试输出
            self.debug_callback(f"发送控制指令: linear={linear_velocity:.2f}, angular={angular_velocity:.2f}, flags={control_flags}, payload={payload}")
            
            return True
            
        except Exception as e:
            self.debug_callback(f"发送命令失败: {e}")
            return False

    def stop(self) -> bool:
        """停止"""
        return self.send_velocity_command(0.0, 0.0)

    def emergency_stop(self) -> bool:
        """紧急停止"""
        self.debug_callback("执行紧急停止")
        return self.send_velocity_command(0.0, 0.0, control_flags=1)

    def get_last_command(self) -> Optional[VelocityCommand]:
        """获取最后一个命令"""
        return self._last_command

    # 便捷控制方法
    def move_forward(self, speed: float = 0.5) -> bool:
        """前进"""
        return self.send_velocity_command(speed, 0.0)

    def move_backward(self, speed: float = 0.5) -> bool:
        """后退"""
        return self.send_velocity_command(-speed, 0.0)

    def turn_left(self, speed: float = 0.5) -> bool:
        """左转"""
        return self.send_velocity_command(0.0, speed)

    def turn_right(self, speed: float = 0.5) -> bool:
        """右转"""
        return self.send_velocity_command(0.0, -speed)