# project/infrastructure/ros/sub.py

"""
ROS 话题订阅器封装

支持多种消息类型的订阅，特别是 BLE (UInt8MultiArray) 类型的解析与缓存，
提供统一的回调机制与缓存管理，让代码更优雅、简洁、易于扩展与阅读。
"""

import rospy
from std_msgs.msg import (
    UInt8MultiArray,
    String,
    Bool,
    Int32,
    Float64,
)

import json
import threading
from collections import deque
from typing import Any, Callable, Deque, Dict, List, Optional, Union

from project.utils.msg_codec import uint8_array_to_json
from project.utils.logging import log


class TopicSubscriber:
    """
    ROS 话题订阅器

    可以订阅多种消息类型，包括 BLE (UInt8MultiArray)、String、Bool、Int32、Float64 等。
    自动将接收到的消息缓存为 Python 字典，并按需调用用户回调函数。
    """

    # 支持的消息类型映射：key 为类型名称，value 为 ROS 消息类
    MSG_TYPES: Dict[str, Any] = {
        'UInt8MultiArray': UInt8MultiArray,
        'String': String,
        'Bool': Bool,
        'Int32': Int32,
        'Float64': Float64,
    }

    def __init__(self) -> None:
        """初始化订阅器管理器"""
        self._subscribers: Dict[str, Dict[str, Any]] = {}
        self._callbacks: Dict[str, Optional[Callable[[Any], None]]] = {}
        self._message_cache: Dict[str, Deque[Dict[str, Any]]] = {}
        self._cache_sizes: Dict[str, int] = {}
        self._lock = threading.RLock()

    def create_subscriber(
        self,
        topic: str,
        msg_type: str,
        callback: Optional[Callable[[Any], None]] = None,
        cache_size: int = 100,
        queue_size: int = 10,
    ) -> bool:
        """
        创建 ROS 订阅器并注册回调

        Args:
            topic: 话题名称
            msg_type: 消息类型名称 (如 'UInt8MultiArray', 'String' 等)
            callback: 用户回调函数，参数为原始 ROS 消息对象
            cache_size: 本地缓存该话题消息的最大条数 (0 表示不缓存)
            queue_size: ROS 队列大小

        Returns:
            True 创建成功；False 失败（类型不支持或 ROS 抛错）
        """
        log.info(f"MSG_TYPES : {self.MSG_TYPES} | msg_type = {msg_type}")
        if msg_type not in self.MSG_TYPES:
            log.error(f"不支持的消息类型: {msg_type}")
            return False

        msg_class = self.MSG_TYPES[msg_type]
        try:
            # 内部回调函数：先缓存，再调用用户回调
            def _internal_cb(msg: Any) -> None:
                self._handle_message(topic, msg, callback)

            with self._lock:
                subscriber = rospy.Subscriber(topic, msg_class, _internal_cb, queue_size=queue_size)

                self._subscribers[topic] = {
                    'subscriber': subscriber,
                    'msg_type': msg_type,
                    'msg_class': msg_class,
                }
                self._callbacks[topic] = callback

                # 初始化缓存队列
                if cache_size > 0:
                    self._message_cache[topic] = deque(maxlen=cache_size)
                    self._cache_sizes[topic] = cache_size

            log.info(f"创建订阅器成功: {topic} ({msg_type})")
            return True

        except Exception as e:
            log.error(f"创建订阅器失败: {topic} - {e}")
            return False

    def _handle_message(self, topic: str, msg: Any, callback: Optional[Callable[[Any], None]]) -> None:
        """
        订阅到消息后的内部处理

        会将消息转换为字典缓存（如果开启缓存），并执行用户回调。
        """
        try:
            with self._lock:
                if topic in self._message_cache:
                    msg_dict = self._msg_to_dict(msg)
                    msg_dict['_timestamp'] = rospy.Time.now().to_sec()
                    self._message_cache[topic].append(msg_dict)

            if callback:
                try:
                    callback(msg)
                except Exception as e:
                    log.error(f"用户回调执行失败: {topic} - {e}")

        except Exception as e:
            log.error(f"处理消息失败: {topic} - {e}")

    def _msg_to_dict(self, msg: Any) -> Dict[str, Any]:
        """
        将 ROS 消息对象转换为 Python 字典

        对于 UInt8MultiArray，会尝试解析为 JSON 字典；对其他类型则读取常用字段。
        """
        try:
            result: Dict[str, Any] = {}

            if isinstance(msg, UInt8MultiArray):
                # 将 uint8 数组转换为 JSON 字典 (若无法解析，则返回原始 bytes 转 hex）
                parsed = uint8_array_to_json(list(msg.data))
                if parsed is not None:
                    result.update(parsed)
                else:
                    # fallback: 原始二进制数据以十六进制字符串存储
                    result['raw_data_hex'] = msg.data.tobytes().hex()

            elif isinstance(msg, String):
                # String 类型 data 字段，尝试解析 JSON 字符串
                try:
                    result.update(json.loads(msg.data))
                except Exception:
                    result['data'] = msg.data

            elif isinstance(msg, (Bool, Int32, Float64)):
                # 基础类型直接读取 data 字段
                result['data'] = msg.data

            else:
                # 其他自定义消息：尝试获取常见属性
                for attr in ['data', 'x', 'y', 'z', 'linear', 'angular', 'pose']:
                    if hasattr(msg, attr):
                        value = getattr(msg, attr)
                        if hasattr(value, '__dict__'):
                            result.update(value.__dict__)
                        else:
                            result[attr] = value

            result['_msg_type'] = type(msg).__name__
            return result

        except Exception as e:
            log.error(f"消息转换为字典失败: {e}")
            return {'_msg_type': type(msg).__name__, '_error': str(e)}

    def get_latest_message(self, topic: str) -> Optional[Dict[str, Any]]:
        """
        获取指定话题的最新缓存消息

        Args:
            topic: 话题名称

        Returns:
            最新消息字典；若无消息或未开启缓存，返回 None
        """
        with self._lock:
            cache = self._message_cache.get(topic)
            if cache and len(cache) > 0:
                return dict(cache[-1])
        return None

    def get_message_history(self, topic: str, count: int = -1) -> List[Dict[str, Any]]:
        """
        获取指定话题的消息历史记录

        Args:
            topic: 话题名称
            count: 返回最近 count 条消息；若 count <= 0，则返回全部缓存

        Returns:
            消息字典列表，若无缓存或话题不存在，返回空列表
        """
        with self._lock:
            cache = self._message_cache.get(topic)
            if not cache:
                return []

            if count > 0:
                return list(cache)[-count:]
            return list(cache)

    def wait_for_message(self, topic: str, timeout: float = 5.0) -> Optional[Dict[str, Any]]:
        """
        阻塞等待指定话题的下一条消息（基于缓存计数检测）

        Args:
            topic: 话题名称
            timeout: 最长等待时间（秒）

        Returns:
            收到的最新消息字典；超时返回 None 或若未订阅该话题也返回 None
        """
        if topic not in self._subscribers:
            log.error(f"话题订阅器不存在: {topic}")
            return None

        start = rospy.Time.now().to_sec()
        original_len = len(self._message_cache.get(topic, ()))

        while rospy.Time.now().to_sec() - start < timeout:
            with self._lock:
                current_len = len(self._message_cache.get(topic, ()))
            if current_len > original_len:
                return self.get_latest_message(topic)
            rospy.sleep(0.01)

        log.warning(f"等待消息超时: {topic}")
        return None

    def set_callback(self, topic: str, callback: Optional[Callable[[Any], None]]) -> bool:
        """
        更新已订阅话题的用户回调函数

        Args:
            topic: 话题名称
            callback: 新的回调函数

        Returns:
            True 更新成功；False 话题未订阅
        """
        with self._lock:
            if topic in self._subscribers:
                self._callbacks[topic] = callback
                self._subscribers[topic]['subscriber']  # ROS Subscriber 不需重新创建
                log.info(f"更新回调函数: {topic}")
                return True

        log.error(f"话题订阅器不存在: {topic}")
        return False

    def get_subscriber_info(self, topic: str) -> Optional[Dict[str, Any]]:
        """
        获取已创建订阅器的基本信息

        Args:
            topic: 话题名称

        Returns:
            包含 msg_type、缓存大小、已缓存消息数量等信息的字典；若不存在返回 None
        """
        with self._lock:
            info = self._subscribers.get(topic)
            if not info:
                return None

            return {
                'topic': topic,
                'msg_type': info['msg_type'],
                'cache_size': self._cache_sizes.get(topic, 0),
                'cached_messages': len(self._message_cache.get(topic, [])),
            }

    def list_subscribers(self) -> Dict[str, str]:
        """
        列出所有当前订阅的主题及其消息类型

        Returns:
            字典 { topic: msg_type }
        """
        with self._lock:
            return {topic: info['msg_type'] for topic, info in self._subscribers.items()}

    def remove_subscriber(self, topic: str) -> bool:
        """
        移除某个话题订阅器，取消 ROS 订阅并清理缓存

        Args:
            topic: 话题名称

        Returns:
            True 移除成功；False 话题不存在
        """
        with self._lock:
            info = self._subscribers.get(topic)
            if not info:
                return False

            try:
                info['subscriber'].unregister()
            except Exception:
                pass

            self._subscribers.pop(topic, None)
            self._callbacks.pop(topic, None)
            self._message_cache.pop(topic, None)
            self._cache_sizes.pop(topic, None)

            log.info(f"移除订阅器: {topic}")
            return True

    def clear_cache(self, topic: Optional[str] = None) -> None:
        """
        清空缓存

        Args:
            topic: 指定话题清空该话题缓存；若为 None，则清空所有话题缓存
        """
        with self._lock:
            if topic:
                if topic in self._message_cache:
                    self._message_cache[topic].clear()
                    log.info(f"清空缓存: {topic}")
            else:
                for cache in self._message_cache.values():
                    cache.clear()
                log.info("清空所有缓存")

    def clear_all(self) -> None:
        """
        清除所有订阅器和缓存，取消所有 ROS 订阅
        """
        with self._lock:
            for info in self._subscribers.values():
                try:
                    info['subscriber'].unregister()
                except Exception:
                    pass

            self._subscribers.clear()
            self._callbacks.clear()
            self._message_cache.clear()
            self._cache_sizes.clear()

        log.info("已清除所有订阅器")


# —— 全局便捷调用 —— #
_global_subscriber: Optional[TopicSubscriber] = None


def get_subscriber() -> TopicSubscriber:
    """
    单例模式获取全局 TopicSubscriber 实例
    """
    global _global_subscriber
    if _global_subscriber is None:
        _global_subscriber = TopicSubscriber()
    return _global_subscriber


def subscribe_to_topic(
    topic: str,
    msg_type: str,
    callback: Optional[Callable[[Any], None]] = None,
    cache_size: int = 100,
    queue_size: int = 10,
) -> bool:
    """
    全局便捷函数：订阅指定话题

    Args:
        topic: 话题名称
        msg_type: 消息类型名称 (如 'UInt8MultiArray', 'String' 等)
        callback: 回调函数，参数为 ROS 消息对象
        cache_size: 本地缓存大小
        queue_size: ROS 订阅队列大小

    Returns:
        True 订阅成功；False 失败
    """
    subscriber = get_subscriber()
    return subscriber.create_subscriber(topic, msg_type, callback, cache_size, queue_size)
