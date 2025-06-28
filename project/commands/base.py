"""
抽象命令基类  
- validate():    业务参数校验（可在子类实现）  
- send():        一步下发 + 注册回调 + 超时  
- _handle_response(): 分发 ACK / NACK / EVENT / TIMEOUT
"""
from __future__ import annotations


from abc import ABC, abstractmethod
from typing import Any, Dict, Optional

from project.utils.logging import log
from project.core.command import send_command

class Command(ABC):
    TIMEOUT: float = 100.0        # YAML 读取的超时
    METHOD: str                   # YAML 读取的method
    PRIORITY: int                 # YAML 读取的优先级
    command_name: str             # 必填：底层协议关键字

    def __init__(self, params: Dict[str, Any] | None = None):
        if params is None:
            params = {}
        if not isinstance(params, dict):
            raise TypeError("params must be dict")
        self.params = params
        self._q: Optional[int] = None
        self.validate()

    def build_payload(self) -> dict:
        """
        构造最终下发的 payload,支持被不同域或子类覆写
        """
        # 默认行为（可以是简单透传,也可以 raise NotImplementedError,推荐前者）
        return self.params

    # ---------------- public ----------------
    def send(self, *, timeout: float | None = None) -> int:
        real_timeout = timeout if timeout is not None else self.TIMEOUT
        payload = self.build_payload()
        log.info(f'发送命令开始 - command_name: {self.command_name}, timeout: {real_timeout}, payload: {payload}')
        self._q = send_command(
            payload,
            callback=self._handle_response,
            timeout=real_timeout,
        )
        return 1

    # -------------- 钩子：子类可覆写 --------------
    def validate(self) -> None:
        pass

    def on_ack(self, response: Dict[str, Any]) -> None:
        """第一次 ACK & success=True 回调"""
        log.error("%s q=%s ACK (%.1fs)", self.command_name, self._q, response)

    def on_nack(self, response: Dict[str, Any]) -> None:
        log.error("%s q=%s NACK: %s", self.command_name, self._q, response.get("error"))

    def on_event(self, response: Dict[str, Any]) -> None:
        log.info("%s q=%s EVENT: %s", self.command_name, self._q, response)

    def on_timeout(self, response: Dict[str, Any]) -> None:
        log.error("%s q=%s TIMEOUT (%.1fs)", self.command_name, self._q, self.TIMEOUT)

    # -------------- 内部分发 --------------
    def _handle_response(self, resp: Dict[str, Any]) -> None:
        stat = resp.get("status")
        if stat == "ack":
            return self.on_ack(resp) if resp.get("success") else self.on_nack(resp)
        if stat == "timeout":
            return self.on_timeout(resp)
        return self.on_event(resp)
