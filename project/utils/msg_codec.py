"""
工具函数

主要处理蓝牙消息格式转换和数据处理
"""

import json
import struct
from typing import Dict, List, Any, Optional, Union
from .logging import log

def json_to_uint8_array(data: Union[Dict, str]) -> List[int]:
    """
    将JSON数据转换为UInt8数组
    
    Args:
        data: 字典数据或JSON字符串
        
    Returns:
        List[int]: UInt8数组
    """
    try:
        # 如果是字典，先转为JSON字符串
        if isinstance(data, dict):
            json_str = json.dumps(data, ensure_ascii=False, separators=(',', ':'))
        elif isinstance(data, str):
            # 验证是否为有效JSON
            json.loads(data)  # 验证格式
            json_str = data
        else:
            raise ValueError(f"不支持的数据类型: {type(data)}")
        
        # 转换为UTF-8字节
        json_bytes = json_str.encode('utf-8')
        
        # 转换为UInt8数组
        uint8_array = list(json_bytes)
        
        log.debug(f"JSON转UInt8数组: {len(json_str)}字符 -> {len(uint8_array)}字节")
        return uint8_array
        
    except Exception as e:
        log.error(f"JSON转UInt8数组失败: {e}")
        return []

def uint8_array_to_json(uint8_array: List[int]) -> Optional[Dict]:
    """
    将UInt8数组转换为JSON数据
    
    Args:
        uint8_array: UInt8数组
        
    Returns:
        Dict: 解析后的字典数据，失败返回None
    """
    try:
        # 转换为字节
        json_bytes = bytes(uint8_array)
        
        # 解码为字符串
        json_str = json_bytes.decode('utf-8')
        
        # 解析JSON
        data = json.loads(json_str)
        
        log.debug(f"UInt8数组转JSON: {len(uint8_array)}字节 -> {len(json_str)}字符")
        return data
        
    except Exception as e:
        log.error(f"UInt8数组转JSON失败: {e}")
        return None
