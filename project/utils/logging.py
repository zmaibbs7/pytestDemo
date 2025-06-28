"""
统一日志管理模块

关键特性
--------
1. 线程/多进程安全的单例初始化
2. 控制台彩色输出（自动检测 colorama & TTY）
3. 支持按大小/时间滚动的文件日志，自动创建目录
4. 运行期动态调级、模块级别过滤
5. 函数调用 / 性能监控装饰器
6. 完整类型注解，兼容 Python 3.8+
"""

from __future__ import annotations

import datetime as _dt
import logging
import os
import sys
import threading
import time
from functools import wraps
from logging.handlers import RotatingFileHandler, TimedRotatingFileHandler
from pathlib import Path
from typing import Callable, Optional, Union

# ----------------------------------------------------------------------
# 1. 预定义日志级别
LOG_LEVELS: dict[str, int] = {
    "DEBUG": logging.DEBUG,
    "INFO": logging.INFO,
    "WARNING": logging.WARNING,
    "ERROR": logging.ERROR,
    "CRITICAL": logging.CRITICAL,
}

# ----------------------------------------------------------------------
# 2. 格式化器
class SimpleFormatter(logging.Formatter):
    """示例: [2025‑06‑18 09:12:34.567] [   INFO] [module] message"""

    _FMT = "[%(asctime)s] [%(levelname)8s] [%(name)s] %(message)s"
    _DATEFMT = "%Y-%m-%d %H:%M:%S.%f"

    def __init__(self, colored: bool = False) -> None:
        super().__init__(self._FMT, datefmt=self._DATEFMT)
        self._colored = colored and _supports_color()

    def format(self, record: logging.LogRecord) -> str:
        msg = super().format(record)
        if self._colored:
            msg = _colorize(record.levelno, msg)
        return msg

    def formatTime(self, record: logging.LogRecord, datefmt: Optional[str] = None) -> str:  # noqa: D401
        ts = _dt.datetime.fromtimestamp(record.created)
        s = ts.strftime(datefmt or self._DATEFMT)
        return s[:-3]  # 保留 3 位毫秒


# ----------------------------------------------------------------------
# 3. 内部工具：颜色辅助
def _supports_color() -> bool:
    return sys.stdout.isatty() and ("colorama" in sys.modules or _try_import_colorama())


def _try_import_colorama() -> bool:
    try:
        import colorama  # type: ignore
    except ModuleNotFoundError:
        return False
    else:
        colorama.just_fix_windows_console()
        return True


_COLOR_MAP = {
    logging.DEBUG: 36,     # cyan
    logging.INFO: 32,      # green
    logging.WARNING: 33,   # yellow
    logging.ERROR: 31,     # red
    logging.CRITICAL: 41,  # red background
}


def _colorize(level: int, text: str) -> str:
    code = _COLOR_MAP.get(level)
    return f"\x1b[{code}m{text}\x1b[0m" if code else text


# ----------------------------------------------------------------------
# 4. 日志管理器
class LogManager:
    """集中管理 console / file logger，并提供便捷装饰器"""

    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._setup_done: bool = False
        self._default_level: int = logging.DEBUG
        self._console_handler: Optional[logging.Handler] = None
        self._file_handler: Optional[logging.Handler] = None

    # ---- 初始化 -----------------------------------------------------
    def setup_logging(
        self,
        *,
        console_level: int = logging.DEBUG,
        log_file: Optional[Union[str, os.PathLike[str]]] = None,
        file_level: int = logging.DEBUG,
        max_bytes: int = 10 * 1024 * 1024,
        backup_count: int = 5,
        rotation: str = "size",  # 'size' | 'time'
        encoding: str = "utf-8",
        colored: bool = True,
        force: bool = False,
    ) -> None:
        """初始化根日志。rotation='time' 时按天切分。"""
        with self._lock:
            if self._setup_done and not force:
                return

            root = logging.getLogger()
            # 关闭并移除旧 handler
            for h in root.handlers[:]:
                try:
                    h.flush()
                    h.close()
                finally:
                    root.removeHandler(h)

            fmt = SimpleFormatter(colored=colored)

            # 控制台处理器
            ch = logging.StreamHandler(sys.stdout)
            ch.setLevel(console_level)
            ch.setFormatter(fmt)
            root.addHandler(ch)
            self._console_handler = ch

            # 文件处理器
            fh: Optional[logging.Handler] = None
            if log_file:
                log_path = Path(log_file)
                log_path.parent.mkdir(parents=True, exist_ok=True)

                if rotation == "time":
                    fh = TimedRotatingFileHandler(
                        str(log_path),
                        when="midnight",
                        encoding=encoding,
                        backupCount=backup_count,
                    )
                elif rotation == "size":
                    fh = RotatingFileHandler(
                        str(log_path),
                        mode="a",
                        encoding=encoding,
                        maxBytes=max_bytes,
                        backupCount=backup_count,
                    )
                else:
                    raise ValueError("rotation 参数必须是 'size' 或 'time'")

                fh.setLevel(file_level)
                fh.setFormatter(fmt)
                root.addHandler(fh)
                self._file_handler = fh

            # 根级别设为所有 handler 最低级别
            levels = [console_level, file_level] if fh else [console_level]
            root.setLevel(min(levels))

            self._default_level = console_level
            self._setup_done = True
            logging.getLogger(__name__).info(
                "日志系统初始化完成 (%s)",
                "控制台+文件" if fh else "仅控制台",
            )

    # ---- 便捷接口 ---------------------------------------------------
    def get_logger(self, name: str) -> logging.Logger:
        if not self._setup_done:
            # 惰性初始化：仅控制台 DEBUG
            self.setup_logging(console_level=self._default_level)
        return logging.getLogger(name)

    def set_level(self, level: int, handler_type: str = "all") -> None:
        """动态调整处理器级别: handler_type ∈ {'console','file','all'}"""
        with self._lock:
            root = logging.getLogger()
            for h in root.handlers:
                is_console = (
                    isinstance(h, logging.StreamHandler)
                    and not isinstance(h, (RotatingFileHandler, TimedRotatingFileHandler))
                )
                is_file = isinstance(h, (RotatingFileHandler, TimedRotatingFileHandler))

                if handler_type == "all" or (handler_type == "console" and is_console) or (
                    handler_type == "file" and is_file
                ):
                    h.setLevel(level)

            # 根 logger 级别同步为最小值
            root.setLevel(min(h.level for h in root.handlers))
            if handler_type in {"console", "all"}:
                self._default_level = level

    def add_module_filter(self, module_prefix: str, min_level: int) -> None:
        """限制指定模块前缀的日志最低级别"""

        class _MinLevelFilter(logging.Filter):
            def __init__(self, prefix: str, level: int) -> None:
                super().__init__()
                self.prefix = prefix
                self.level = level

            def filter(self, record: logging.LogRecord) -> bool:
                return not (
                    record.name.startswith(self.prefix) and record.levelno < self.level
                )

        logging.getLogger().addFilter(_MinLevelFilter(module_prefix, min_level))

    # ---- 调试统计 ---------------------------------------------------
    def get_stats(self) -> dict[str, object]:
        root = logging.getLogger()
        return {
            "handlers": [type(h).__name__ for h in root.handlers],
            "root_level": logging.getLevelName(root.level),
            "default_console_level": logging.getLevelName(self._default_level),
            "setup_done": self._setup_done,
            "console_level": (
                logging.getLevelName(self._console_handler.level)
                if self._console_handler
                else None
            ),
            "file_level": (
                logging.getLevelName(self._file_handler.level)
                if self._file_handler
                else None
            ),
        }


# ----------------------------------------------------------------------
# 5. 装饰器
def log_function_call(logger_name: Optional[str] = None):
    """记录同步函数调用全过程（进入 / 返回 / 异常）"""

    def decorator(func: Callable[..., object]):
        @wraps(func)
        def wrapper(*args, **kwargs):  # type: ignore[override]
            log = logManager.get_logger(logger_name or func.__module__)
            log.debug("调用: %s.%s", func.__module__, func.__name__)
            try:
                result = func(*args, **kwargs)
                log.debug("返回: %s.%s -> %r", func.__module__, func.__name__, result)
                return result
            except Exception:
                log.exception("异常: %s.%s", func.__module__, func.__name__)
                raise

        return wrapper

    return decorator

def log_performance(
    logger_name: Optional[str] = None,
    threshold_ms: float = 100.0,
):
    """监控同步函数耗时，超过阈值则 WARNING，否则 DEBUG"""

    def decorator(func: Callable[..., object]):
        @wraps(func)
        def wrapper(*args, **kwargs):  # type: ignore[override]
            log = logManager.get_logger(logger_name or func.__module__)
            start = time.perf_counter()
            try:
                return func(*args, **kwargs)
            finally:
                elapsed = (time.perf_counter() - start) * 1000
                msg = "%s.%s 耗时 %.1f ms" % (func.__module__, func.__name__, elapsed)
                if elapsed > threshold_ms:
                    log.warning(msg)
                else:
                    log.debug(msg)

        return wrapper

    return decorator

# ----------------------------------------------------------------------
# 6. 实例化全局
logManager = LogManager()

def setup_logging(
    level: int = logging.INFO,
    log_file: Optional[Union[str, os.PathLike[str]]] = None,
    file_level: int = logging.DEBUG,
    **kwargs,
) -> None:
    """顶层快捷初始化；其余参数透传"""
    logManager.setup_logging(
        console_level=level,
        log_file=log_file,
        file_level=file_level,
        **kwargs,
    )

# ----------------------------------------------------------------------
# 7. 全局日志代理
class LoggerProxy:
    """使用 `log.info(...)` 即可根据调用者模块获得 logger"""

    def __getattr__(self, name: str) -> Callable[..., None]:
        caller_mod = sys._getframe(1).f_globals.get("__name__", "__main__")
        logger = logManager.get_logger(caller_mod)
        return getattr(logger, name)


log = LoggerProxy()