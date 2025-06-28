# /project/core/command.py
import json
import threading
from itertools import count
from typing import Protocol, Callable, Dict, Any, Optional

import rospy
from std_msgs.msg import UInt8MultiArray, String

from project.infrastructure.ros.pub import publish_ble
from project.core.timerManager import TimeManager
from project.utils.logging import log

# 全局递增请求 ID 生成器
_request_id_counter = count(1)

def _get_next_request_id() -> int:
    """返回全局递增的请求 ID"""
    return next(_request_id_counter)

class CommandPublisher(Protocol):
    """命令发布接口 - 由基础设施层实现"""
    def publish_command(self, payload: str) -> bool:
        ...

class DefaultCommandPublisher:
    """
    默认命令发布器：使用 ROS pub.py 中的 publish_ble 函数
    """
    def __init__(self, topic: str = "/ble/cmd"):
        self.topic = topic

    def publish_command(self, payload: str) -> bool:
        return publish_ble(self.topic, payload)

class CommandManager:
    """命令管理器,提供发送与接收命令的统一入口"""
    def __init__(
        self,
        publisher: CommandPublisher = DefaultCommandPublisher(),
        default_timeout: float = 5.0
    ):
        log.info(f"--init-- CommandManager")
        self._publisher = publisher
        self._timeout = default_timeout
        self._callbacks: Dict[int, Callable[[Dict[str, Any]], None]] = {}
        self._lock = threading.Lock()

    def send_command(
        self,
        payload: Dict[str, Any],
        callback: Callable[[Dict[str, Any]], None],
        timeout: Optional[float] = None
    ) -> int:
        """
        生成唯一请求 ID 并发送命令,注册响应回调及超时处理。
        """
        q = _get_next_request_id()
        payload["q"] = q

        data_str = json.dumps(payload, ensure_ascii=False)

        log.info("[command] 发送命令: %s", payload)
        success = self._publisher.publish_command(data_str)
        if not success:
            log.error("[command] 发布命令失败: %s, q=%d", payload, q)
            raise RuntimeError(f"[CommandManager] 发布命令失败: {payload}, q={q}")

        with self._lock:
            self._callbacks[q] = callback

        def on_timeout():
            with self._lock:
                cb = self._callbacks.pop(q, None)
            if cb:
                log.warning("[command] 命令 %d 超时", q)
                cb({"q": q, "status": "timeout"})

        TimeManager.instance().addTimeout(q, on_timeout, timeout or self._timeout)
        return q

    def on_ros_msg(self, msg: String) -> None:
        """
        ROS 订阅回调：解析 JSON 并分发到对应的回调函数。
        """
        try:
            data = json.loads(msg.data)
        except Exception:
            log.error("[command] 无效 ROS 消息格式: %s", msg.data)
            return

        q = data.get("q")
        if q is None:
            return

        with self._lock:
            cb = self._callbacks.pop(q, None)
        if cb:
            log.info("[command] 收到命令 q: %d 响应: %s", q, data)
            try:
                cb(data)
            except Exception as e:
                log.error("[command] 回调执行出错: %s", e)

_CM: Optional[CommandManager] = None

# 对外暴露统一接口
def get_command_manager() -> CommandManager:
    """返回全局 CommandManager 实例（若未创建则惰性初始化）。"""
    global _CM
    if _CM is None:
        _CM = CommandManager()
    return _CM

def send_command(
    payload: Dict[str, Any],
    callback: Callable[[Dict[str, Any]], None],
    timeout: Optional[float] = None,
) -> int:
    return get_command_manager().send_command(payload, callback, timeout)