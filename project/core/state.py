# project/core/state.py
"""
运行时状态管理模块
职责：
1. 更新机器人状态（心跳、位姿、割草进度）
2. 提供单例方式获取当前状态
3. 线程安全缓存 + 监听器接口
"""

from __future__ import annotations

import dataclasses
import threading
from dataclasses import dataclass, field
from enum import IntEnum, unique
from typing import Callable, Dict, List, Type, Union, TypeAlias   

from project.utils.logging import log

# --------------------------------------------------------------------------- #
#  枚举定义
# --------------------------------------------------------------------------- #
@unique
class StateType(IntEnum):
    STATE_INIT = 0
    STATE_IDLE = 1
    STATE_PAIR_NET = 2
    STATE_REMOTE = 3
    STATE_MAPPING = 4
    STATE_WORK = 5
    STATE_TYPE_MAX = 16
    UNKNOWN = 255  # 默认

@unique
class MappingStateType(IntEnum):
    MAPPING_STATE_IDLE = StateType.STATE_TYPE_MAX + 1  # 17
    MAPPING_STATE_PREPARE = 18
    MAPPING_STATE_REMOTE = 19
    MAPPING_STATE_REGION = 20
    MAPPING_STATE_ISLAND = 21
    MAPPING_STATE_PATH = 22
    MAPPING_STATE_AREA = 23
    MAPPING_STATE_EDIT = 24
    MAPPING_STATE_GENERATING = 25
    MAPPING_STATE_FAILED = 26
    MAPPING_STATE_EXIT = 27
    MAPPING_STATE_ERASE = 28
    MAPPING_STATE_AUTO = 29
    MAPPING_TYPE_MAX = StateType.STATE_TYPE_MAX + 16  # 32
    UNKNOWN = 255

@unique
class WorkingStateType(IntEnum):
    WORKING_STATE_IDLE = MappingStateType.MAPPING_TYPE_MAX + 1  # 33
    WORKING_STATE_START = 34
    WORKING_STATE_RUNNING = 35
    WORKING_STATE_PAUSE = 36
    WORKING_STATE_FINISH = 37
    WORKING_STATE_FAILED = 38
    WORKING_STATE_EXIT = 39
    WORKING_STATE_DOCK = 40
    WORKING_STATE_MAX = MappingStateType.MAPPING_TYPE_MAX + 16  # 48
    UNKNOWN = 255

SUB_STATE_MAP: Dict[StateType, Type[IntEnum]] = {
    StateType.STATE_MAPPING: MappingStateType,
    StateType.STATE_WORK: WorkingStateType,
}
SubState = Union[MappingStateType, WorkingStateType]

# --------------------------------------------------------------------------- #
#  数据类
# --------------------------------------------------------------------------- #
@dataclass(frozen=True, slots=True)
class BatteryInfo:
    level: float = 0.0          # 0–100 %
    is_charging: bool = False


@dataclass(frozen=True, slots=True)
class LocationInfo:
    x: float = 0.0              # m
    y: float = 0.0              # m
    theta: float = 0.0          # rad


@dataclass(frozen=True, slots=True)
class VelocityInfo:
    linear_x: float = 0.0       # m/s
    angular_z: float = 0.0      # rad/s

@dataclass(frozen=True, slots=True)
class RobotState:
    """机器人完整状态"""
    status: StateType = StateType.UNKNOWN
    sub_status: SubState = MappingStateType.UNKNOWN
    battery: BatteryInfo = field(default_factory=BatteryInfo)
    location: LocationInfo = field(default_factory=LocationInfo)
    velocity: VelocityInfo = field(default_factory=VelocityInfo)
    
    def __str__(self) -> str:
        return (
            f"RobotState<"
            f"status={self.status.name}({self.status.value}), "
            f"sub_status={self.sub_status.name}({self.sub_status.value}), "
            f"battery={self.battery.level:.1f}%{'(charging)' if self.battery.is_charging else ''}, "
            f"location=({self.location.x:.2f}, {self.location.y:.2f}, {self.location.theta:.2f}), "
            f"velocity=({self.velocity.linear_x:.2f}, {self.velocity.angular_z:.2f}), "
            f">"
        )
# --------------------------------------------------------------------------- #
#  状态管理器（线程安全单例）
# --------------------------------------------------------------------------- #
Listener: TypeAlias = Callable[[RobotState, RobotState], None]
class StateManager:
    """负责更新 / 查询机器人状态"""

    _instance: "StateManager | None" = None
    _instance_lock = threading.Lock()

    def __new__(cls) -> "StateManager":  # noqa: D401
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self) -> None:
        if getattr(self, "_inited", False):
            return
        self._inited = True

        self._state_lock = threading.RLock()
        self._current_state: RobotState = RobotState()
        self._listeners: list[Listener] = []

    # ------------------------------------------------------------------- #
    #  更新入口
    # ------------------------------------------------------------------- #
    def on_heartbeat_msg(self, data: List[int]) -> None:
        """外部将 UInt8MultiArray → List[int] 后调用此函数"""
        if len(data) != 21:
            return # 心跳帧长度不对，忽略
        try:
            new_state = self.parse_heartbeat_msg(self._current_state, data)
        except Exception as exc:  # noqa: BLE001
            log.info("Failed to parse heartbeat: %s", exc)
            return
        log.info(f'new_state: {new_state}')
        with self._state_lock:
            old_state = self._current_state
            if new_state == old_state:
                return  # 状态未变化，无需更新
            log.info(f'new_state: {new_state}')
            self._replace_state(new_state)

    def on_pose_msg(
        self,
        x: float,
        y: float,
        theta: float,
        linear_x: float | None = None,
        angular_z: float | None = None,
    ) -> None:
        """位姿增量更新"""
        with self._state_lock:
            old = self._current_state
            vel = VelocityInfo(
                linear_x=linear_x if linear_x is not None else old.velocity.linear_x,
                angular_z=angular_z if angular_z is not None else old.velocity.angular_z,
            )
            new_state = dataclasses.replace(
                old,
                location=LocationInfo(x=x, y=y, theta=theta),
                velocity=vel
            )
        self._replace_state(new_state)

    # ------------------------------------------------------------------- #
    #  查询接口
    # ------------------------------------------------------------------- #
    def get_current_state(self) -> RobotState:
        with self._state_lock:
            return self._current_state

    def is_charging(self) -> bool:
        return self.get_current_state().battery.is_charging

    def is_moving(self) -> bool:
        st = self.get_current_state()
        return abs(st.velocity.linear_x) > 0.01 or abs(st.velocity.angular_z) > 0.01

    # ------------------------------------------------------------------- #
    #  监听器管理
    # ------------------------------------------------------------------- #
    def add_listener(self, fn: Listener) -> None:
        with self._state_lock:
            if fn not in self._listeners:
                self._listeners.append(fn)

    def remove_listener(self, fn: Listener) -> None:
        with self._state_lock:
            if fn in self._listeners:
                self._listeners.remove(fn)

    # ------------------------------------------------------------------- #
    #  内部工具
    # ------------------------------------------------------------------- #
    def _replace_state(self, new_state: RobotState) -> None:
        with self._state_lock:
            old_state = self._current_state
            self._current_state = new_state

        for fn in self._listeners.copy():
            try:
                fn(old_state, new_state)
            except Exception:  # noqa: BLE001
                log.exception("State listener raised")

    def parse_heartbeat_msg(self, prev_state: RobotState, raw: List[int]) -> RobotState:
        """
        心跳帧更新入口：外部订阅者将 `UInt8MultiArray` 转为 `List[int]` 后调用本函数
        """
        if len(raw) <= 20:
            raise ValueError(f"heartbeat too short: {len(raw)}")
        if raw[0] != 0x01 or raw[-1] != 0xCE:
            raise ValueError("invalid frame header/tail")

        FRAME_HEADER = 1   # 丢弃 0x01 0xCE
        data = raw[FRAME_HEADER:]
        log.info(f'heartbeat data: {data}')

        # 1-10: error codes —— 按要求保留但暂未写入 RobotState
        # error_codes = data[1:11]

        # Battery at index 11
        b = data[11]
        battery = BatteryInfo(level=b & 0x7F, is_charging=bool(b & 0x80))

        # Main / Sub
        log.info(f'main_state: {data[12]} sub_state: {data[13]}')
        main_state = StateType(data[12])
        sub_state = self.parse_sub_state(main_state, data[13])

        # —— 增量更新 —— #
        return dataclasses.replace(
            prev_state,
            status=main_state,
            sub_status=sub_state,
            battery=battery
        )

    def parse_sub_state(self, main: StateType, raw_value: int) -> SubState:
        """
        根据主状态挑选对应子枚举；若不匹配或非法值 -> UNKNOWN
        """
        enum_cls = SUB_STATE_MAP.get(main)
        if enum_cls is None:
            return MappingStateType.UNKNOWN  # 主状态无子状态
        try:
            return enum_cls(raw_value)
        except ValueError:
            return enum_cls.UNKNOWN


# --------------------------------------------------------------------------- #
#  便捷全局函数
# --------------------------------------------------------------------------- #
def get_state_manager() -> StateManager:
    return StateManager()


def get_robot_state() -> RobotState:
    return get_state_manager().get_current_state()
