import time
import threading
from typing import Callable, List, Sequence, Any
from unittest.mock import Mock


class TimeoutError(Exception):
    """等待超时"""


class _BaseHandle:
    """所有 Handle 的基类：实现 is_satisfied() / wait()"""

    def __init__(self, predicate: Callable[[], bool], desc: str, poll: float):
        self._predicate = predicate  # 判断条件是否满足的函数
        self._desc = desc           # 描述信息
        self._poll = poll           # 轮询间隔

    # ---------- 用户接口 ----------
    def is_satisfied(self) -> bool:
        return self._predicate()

    def wait(self, timeout: float | None = None) -> None:
        """阻塞直到 predicate 条件满足；超时则抛异常"""
        deadline = None if timeout is None else time.time() + timeout
        while not self._predicate():
            if deadline and time.time() >= deadline:
                raise TimeoutError(f"timeout waiting for: {self._desc}")
            time.sleep(self._poll)

    # 用于 manager.wait 的描述
    @property
    def desc(self) -> str:
        return self._desc


class _AttrHandle(_BaseHandle):
    pass  # 没有额外字段


class _CallableHandle(_BaseHandle):
    """对函数做 wrap,暴露 `fn` 给业务代码"""

    def __init__(
        self,
        original_fn: Callable,
        desc: str,
        poll: float,
    ):
        self._mock = Mock(wraps=original_fn)

        def wrapper(*args, **kwargs):
            return self._mock(*args, **kwargs)

        self.fn = wrapper
        super().__init__(predicate=lambda: self._mock.called, desc=desc, poll=poll)


class AsyncObserver:
    """
    1. watch_attr(obj, attr, predicate=..., desc="...")   -> _AttrHandle
    2. watch_callable(fn)                                 -> _CallableHandle
       - 业务侧使用 handle.fn
    3. watch_predicate(custom_fn, desc="...")             -> _BaseHandle
    4. wait(handles=[h1, h2], timeout=...)
       - 若 handles=None，等 *全部*；否则只等给定子集
    """

    def __init__(self, poll_interval: float = 0.5):
        self._poll = poll_interval # 轮询间隔 默认为 500毫秒
        self._handles: List[_BaseHandle] = []

    # ---------- watch ----------
    def watch_attr(
        self,
        obj: Any,
        attr: str,
        predicate: Callable[[Any], bool] = lambda v: bool(v),
        *,
        desc: str | None = None,
    ):
        desc = desc or f"{obj}.{attr}"
        handle = _AttrHandle(
            predicate=lambda: predicate(getattr(obj, attr)),
            desc=desc,
            poll=self._poll,
        )
        self._handles.append(handle)
        return handle

    def watch_callable(self, fn: Callable, *, desc: str | None = None):
        desc = desc or f"{fn.__name__} called"
        handle = _CallableHandle(fn, desc, poll=self._poll)
        self._handles.append(handle)
        return handle

    def watch_predicate(self, predicate: Callable[[], bool], desc: str):
        handle = _BaseHandle(predicate, desc, poll=self._poll)
        self._handles.append(handle)
        return handle

    # ---------- wait ----------
    def wait(self, handles: Sequence[_BaseHandle] | None = None, *, timeout: float | None = None):
        """
        等待指定 handles（缺省表示全部）均满足。
        """
        targets = self._handles if handles is None else handles
        deadline = None if timeout is None else time.time() + timeout

        while True:
            pending = [h for h in targets if not h.is_satisfied()]
            if not pending:
                return
            if deadline and time.time() >= deadline:
                names = ", ".join(h.desc for h in pending)
                raise TimeoutError(f"timeout; pending: {names}")
            time.sleep(self._poll)
