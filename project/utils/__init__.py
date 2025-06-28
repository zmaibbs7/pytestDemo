# project/utils/__init__.py

"""
通用工具模块

提供日志、ROS工具等通用功能
"""

from .logging import (
    # 日志初始化与获取
    setup_logging,
    log,
    logManager,
    # 装饰器
    log_function_call,
    log_performance,
    # 日志级别常量字典
    LOG_LEVELS,
)
from .msg_codec import json_to_uint8_array, uint8_array_to_json
from .async_observer import AsyncObserver
    

# 日志级别宏（便于调用）
LOG_DEBUG = LOG_LEVELS['DEBUG']
LOG_INFO = LOG_LEVELS['INFO']
LOG_WARNING = LOG_LEVELS['WARNING']
LOG_ERROR = LOG_LEVELS['ERROR']
LOG_CRITICAL = LOG_LEVELS['CRITICAL']

__all__ = [
    # === module: 日志 ====
    # 核心接口
    'setup_logging',
    'log',
    "logManager"
    # 装饰器
    'log_function_call',
    'log_performance',
    # 日志级别常量
    'LOG_DEBUG',
    'LOG_INFO',
    'LOG_WARNING',
    'LOG_ERROR',
    'LOG_CRITICAL',
    # === module: 编码器 ====
    'json_to_uint8_array',
    'uint8_array_to_json',
    # === module: 异步通用观察器 ====
    'AsyncObserver'
]
