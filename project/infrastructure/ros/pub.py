# project/infrastructure/ros/pub.py
import rospy
import json
from std_msgs.msg import UInt8MultiArray, String
from typing import Any, Dict, Optional, Union, List

# 从 utils/msg_codec 导入
from project.utils.msg_codec import json_to_uint8_array, uint8_array_to_json
from project.utils.logging import log

# 全局的 sequence_id 生成器
_sequence_id = 0


def next_sequence_id() -> int:
    """
    全局自增序列 ID，每调用一次便自增 1，返回新值
    """
    global _sequence_id
    _sequence_id += 1
    return _sequence_id


class TopicPublisher:
    """
    ROS 话题发布器 - 支持发布 BLE（UInt8MultiArray）或 JSON（String）消息
    同时封装了一个便捷的 '命令' 发布方法 publish_order()
    """

    def __init__(self) -> None:
        # 存储已创建的 publisher：{ topic: { 'publisher': rospy.Publisher, 'msg_type': 消息类型, 'msg_count': int } }
        self._publishers: Dict[str, Dict[str, Any]] = {}

    def create_publisher(self,
                         topic: str,
                         msg_type: type = UInt8MultiArray,
                         queue_size: int = 10) -> bool:
        """
        创建一个 ROS Publisher 并缓存到内部字典。
        :param topic: 话题名称
        :param msg_type: 消息类型，支持 UInt8MultiArray 或 String
        :param queue_size: 队列长度
        :return: True 表示创建成功，False 表示失败（可能是已经存在或 API 抛错）
        """
        if topic in self._publishers:
            log.warning(f"Publisher 已存在，跳过创建: {topic}")
            return True

        try:
            publisher = rospy.Publisher(topic, msg_type, queue_size=queue_size)
            self._publishers[topic] = {
                'publisher': publisher,
                'msg_type': msg_type,
                'msg_count': 0
            }
            log.info(f"创建发布器成功: {topic} (类型: {msg_type.__name__})")
            return True
        except Exception as e:
            log.error(f"创建发布器失败: {topic} - {e}")
            return False

    def get_publisher_by_topic(self,
                               topic: str,
                               msg_type: type) -> Optional[rospy.Publisher]:
        """
        根据话题和消息类型获取已缓存的 publisher。
        若话题未创建或类型不匹配，则返回 None。
        """
        info = self._publishers.get(topic)
        if info and info['msg_type'] == msg_type:
            return info['publisher']
        log.error(f"未找到匹配的发布器: {topic} (期望类型: {msg_type.__name__})")
        return None

    def publish_ble(self, topic: str, data: Union[Dict, str]) -> bool:
        """
        发布 BLE 数据：先将 JSON/Dict 转为 UInt8 数组，再封装为 UInt8MultiArray 并发布
        :param topic: 话题名称
        :param data: 字典 或 JSON 字符串
        :return: True 成功，False 失败
        """
        print(f"publish_ble {topic}")
        publisher = self.get_publisher_by_topic(topic, UInt8MultiArray)
        if publisher is None:
            return False

        uint8_array = json_to_uint8_array(data)
        if not uint8_array:
            log.error(f"BLE 消息编码失败: {topic}")
            return False

        try:
            msg = UInt8MultiArray(data=uint8_array)
            publisher.publish(msg)
            self._publishers[topic]['msg_count'] += 1
            print(f"发布 BLE 消息成功: {topic}，字节长度: {len(uint8_array)}")
            log.debug(f"发布 BLE 消息成功: {topic}，字节长度: {len(uint8_array)}")
            return True
        except Exception as e:
            log.error(f"发布 BLE 消息失败: {topic} - {e}")
            return False

    def publish_json(self, topic: str, data: Union[Dict, str]) -> bool:
        """
        发布 JSON 数据：先将 Dict 转为 JSON 字符串，再封装为 String 发布
        :param topic: 话题名称
        :param data: 字典 或 JSON 字符串
        :return: True 成功，False 失败
        """
        publisher = self.get_publisher_by_topic(topic, String)
        if publisher is None:
            log.error(f"未找到 JSON 发布器: {topic}")
            return False

        try:
            if isinstance(data, dict):
                json_str = json.dumps(data, ensure_ascii=False)
            else:
                # 如果 data 是字符串，则认为它已经是合法的 JSON
                json_str = data
            msg = String(data=json_str)
            publisher.publish(msg)
            self._publishers[topic]['msg_count'] += 1
            log.debug(f"发布 JSON 消息成功: {topic}")
            return True
        except Exception as e:
            log.error(f"发布 JSON 消息失败: {topic} - {e}")
            return False

    def publish_order(self,
                      topic: str,
                      order: int,
                      params: Optional[Dict] = None,
                      priority: int = 0) -> bool:
        """
        发布“命令”消息（默认使用 BLE 格式）。
        消息结构: {"q": sequence_id, "m": "a", "p": priority, "o": order, "d": params}
        :param topic: 话题名称（通常是 BLE 相关的 topic，例如 "/ble/cmd"）
        :param order: 命令编号
        :param params: 可选的参数字典
        :param priority: 命令优先级，默认为 0
        :return: True 成功，False 失败
        """
        seq_id = next_sequence_id()
        payload = {
            "q": seq_id,
            "m": "a",
            "p": priority,
            "o": order,
            "d": params or {}
        }
        log.debug(f"准备发布命令: topic={topic}, payload={payload}")
        return self.publish_ble(topic, payload)

    def get_publisher_stats(self, topic: str) -> Optional[Dict[str, Any]]:
        """
        获取指定话题的发布统计信息，包括：已经发布的消息数量、消息类型、是否 active
        :param topic: 话题名称
        :return: 如果存在，则返回 dict；否则返回 None
        """
        info = self._publishers.get(topic)
        if info:
            return {
                'topic': topic,
                'message_count': info['msg_count'],
                'message_type': info['msg_type'].__name__,
                'publisher_active': True
            }
        return None

    def list_publishers(self) -> Dict[str, Dict[str, Any]]:
        """
        列出所有已创建的发布器及其统计信息
        """
        result: Dict[str, Dict[str, Any]] = {}
        for topic, info in self._publishers.items():
            result[topic] = {
                'message_count': info['msg_count'],
                'message_type': info['msg_type'].__name__,
                'active': True
            }
        return result

    def remove_publisher(self, topic: str) -> bool:
        """
        删除指定的话题发布器
        :param topic: 话题名称
        :return: True 表示删除成功（之前存在），False 表示该话题不存在
        """
        if topic in self._publishers:
            del self._publishers[topic]
            log.info(f"移除发布器: {topic}")
            return True
        return False

    def clear_all(self) -> None:
        """
        清空所有缓存的发布器
        """
        self._publishers.clear()
        log.info("已清除所有发布器")


# —— 全局便捷函数 —— #
_global_publisher: Optional[TopicPublisher] = None


def get_publisher() -> TopicPublisher:
    """
    单例模式获取全局 TopicPublisher 实例
    """
    global _global_publisher
    if _global_publisher is None:
        _global_publisher = TopicPublisher()
    return _global_publisher


def publish_ble(topic: str, data: Union[Dict, str]) -> bool:
    """
    全局便捷函数：先获取全局发布器，再调用 publish_ble()
    """
    log.info(f"=== 全局 publish_ble 调用: topic={topic}, data={data} ===")
    pub = get_publisher()
    return pub.publish_ble(topic, data)


def publish_json(topic: str, data: Union[Dict, str]) -> bool:
    """
    全局便捷函数：先获取全局发布器，再调用 publish_json()
    """
    log.debug(f"=== 全局 publish_json 调用: topic={topic}, data={data} ===")
    pub = get_publisher()
    return pub.publish_json(topic, data)


def publish_order(order: int, params: Optional[Dict] = None, priority: int = 0) -> bool:
    """
    全局便捷函数：自动生成 sequence_id 后发布命令消息到默认 topic "/ble/cmd"
    """
    seq_id = next_sequence_id()
    log.debug(f"=== 全局 publish_order: order={order}, params={params}, seq_id={seq_id} ===")
    pub = get_publisher()
    return pub.publish_order("/ble/cmd", order, params, priority)

def create_topic_publisher(topic: str, 
                          msg_type: type = UInt8MultiArray, 
                          queue_size: int = 10) -> bool:
    """
    全局便捷函数：创建话题发布节点
    先获取全局发布器实例，再调用 create_publisher() 方法
    
    :param topic: 话题名称
    :param msg_type: 消息类型，支持 UInt8MultiArray 或 String，默认为 UInt8MultiArray
    :param queue_size: 队列长度，默认为 10
    :return: True 表示创建成功，False 表示失败
    """
    pub = get_publisher()
    return pub.create_publisher(topic, msg_type, queue_size)