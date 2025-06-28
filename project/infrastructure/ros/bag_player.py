# # project/infrastructure/ros/bag_player.py

# """
# ROS Bag 播放器

# 用于仿真测试的 rosbag 播放功能，支持：
# - 播放指定的 bag 文件
# - 控制播放状态（播放、暂停、停止、快进、慢放）
# - 实时监控播放进度
# - 支持循环播放
# - 话题过滤播放
# - 播放速率控制
# """

# import os
# import sys
# import time
# import threading
# import subprocess
# from typing import Dict, List, Optional, Any, Callable
# from enum import Enum
# from dataclasses import dataclass, field

# import rospy
# from rosbag import Bag
# from std_msgs.msg import Header

# from project.utils.logging import get_logger

# logger = get_logger(__name__)


# class PlaybackState(Enum):
#     """播放状态枚举"""
#     STOPPED = "stopped"
#     PLAYING = "playing"
#     PAUSED = "paused"
#     FINISHED = "finished"
#     ERROR = "error"


# @dataclass
# class BagInfo:
#     """Bag文件信息"""
#     filepath: str
#     duration: float = 0.0
#     message_count: int = 0
#     topics: Dict[str, int] = field(default_factory=dict)
#     start_time: float = 0.0
#     end_time: float = 0.0
#     size_mb: float = 0.0


# @dataclass
# class PlaybackStats:
#     """播放统计信息"""
#     current_time: float = 0.0
#     progress_percent: float = 0.0
#     messages_played: int = 0
#     elapsed_time: float = 0.0
#     playback_rate: float = 1.0


# class BagPlayer:
#     """
#     ROS Bag 播放器
    
#     提供完整的 rosbag 播放控制功能，包括播放、暂停、停止、进度控制等。
#     支持多种播放模式和实时状态监控。
#     """

#     def __init__(self):
#         """初始化播放器"""
#         self._bag_file: Optional[str] = None
#         self._bag_info: Optional[BagInfo] = None
#         self._state: PlaybackState = PlaybackState.STOPPED
#         self._playback_thread: Optional[threading.Thread] = None
#         self._stop_event = threading.Event()
#         self._pause_event = threading.Event()
        
#         # 播放控制参数
#         self._playback_rate: float = 1.0
#         self._loop_playback: bool = False
#         self._start_offset: float = 0.0
#         self._duration_limit: Optional[float] = None
#         self._topic_filter: Optional[List[str]] = None
        
#         # 统计信息
#         self._stats = PlaybackStats()
#         self._start_timestamp: Optional[float] = None
        
#         # 回调函数
#         self._state_callbacks: List[Callable[[PlaybackState], None]] = []
#         self._progress_callbacks: List[Callable[[PlaybackStats], None]] = []
        
#         self._lock = threading.RLock()

#     def load_bag(self, bag_file: str) -> bool:
#         """
#         加载 bag 文件并获取基本信息
        
#         Args:
#             bag_file: bag文件路径
            
#         Returns:
#             True 加载成功；False 失败
#         """
#         try:
#             if not os.path.exists(bag_file):
#                 logger.error(f"Bag文件不存在: {bag_file}")
#                 return False
            
#             # 获取文件大小
#             file_size = os.path.getsize(bag_file) / (1024 * 1024)  # MB
            
#             with Bag(bag_file, 'r') as bag:
#                 # 获取基本信息
#                 info = BagInfo(
#                     filepath=bag_file,
#                     duration=bag.get_end_time() - bag.get_start_time(),
#                     start_time=bag.get_start_time(),
#                     end_time=bag.get_end_time(),
#                     size_mb=file_size
#                 )
                
#                 # 统计话题和消息数量
#                 topic_counts = {}
#                 total_messages = 0
                
#                 for topic, msg, t in bag.read_messages():
#                     if topic not in topic_counts:
#                         topic_counts[topic] = 0
#                     topic_counts[topic] += 1
#                     total_messages += 1
                
#                 info.topics = topic_counts
#                 info.message_count = total_messages
            
#             with self._lock:
#                 self._bag_file = bag_file
#                 self._bag_info = info
#                 self._state = PlaybackState.STOPPED
#                 self._reset_stats()
            
#             logger.info(f"成功加载Bag文件: {bag_file}")
#             logger.info(f"  时长: {info.duration:.2f}s")
#             logger.info(f"  消息数: {info.message_count}")
#             logger.info(f"  话题数: {len(info.topics)}")
#             logger.info(f"  文件大小: {info.size_mb:.2f}MB")
            
#             return True
            
#         except Exception as e:
#             logger.error(f"加载Bag文件失败: {bag_file} - {e}")
#             return False

#     def play(self, 
#              rate: float = 1.0,
#              loop: bool = False,
#              start_offset: float = 0.0,
#              duration: Optional[float] = None,
#              topics: Optional[List[str]] = None) -> bool:
#         """
#         开始播放
        
#         Args:
#             rate: 播放速率倍数 (1.0为正常速度)
#             loop: 是否循环播放
#             start_offset: 开始播放的时间偏移(秒)
#             duration: 播放时长限制(秒)，None表示播放到结束
#             topics: 要播放的话题列表，None表示播放所有话题
            
#         Returns:
#             True 开始播放成功；False 失败
#         """
#         with self._lock:
#             if self._bag_file is None:
#                 logger.error("未加载Bag文件")
#                 return False
            
#             if self._state == PlaybackState.PLAYING:
#                 logger.warning("已在播放中")
#                 return True
            
#             # 设置播放参数
#             self._playback_rate = max(0.1, min(10.0, rate))  # 限制在0.1x-10x之间
#             self._loop_playback = loop
#             self._start_offset = max(0.0, start_offset)
#             self._duration_limit = duration
#             self._topic_filter = topics
            
#             # 重置事件和统计
#             self._stop_event.clear()
#             self._pause_event.clear()
#             self._reset_stats()
            
#             # 启动播放线程
#             self._playback_thread = threading.Thread(target=self._playback_worker)
#             self._playback_thread.daemon = True
#             self._playback_thread.start()
            
#             self._set_state(PlaybackState.PLAYING)
#             logger.info(f"开始播放: 速率={rate}x, 循环={loop}")
            
#             return True

#     def pause(self) -> bool:
#         """暂停播放"""
#         with self._lock:
#             if self._state != PlaybackState.PLAYING:
#                 return False
            
#             self._pause_event.set()
#             self._set_state(PlaybackState.PAUSED)
#             logger.info("播放已暂停")
#             return True

#     def resume(self) -> bool:
#         """恢复播放"""
#         with self._lock:
#             if self._state != PlaybackState.PAUSED:
#                 return False
            
#             self._pause_event.clear()
#             self._set_state(PlaybackState.PLAYING)
#             logger.info("播放已恢复")
#             return True

#     def stop(self) -> bool:
#         """停止播放"""
#         with self._lock:
#             if self._state == PlaybackState.STOPPED:
#                 return True
            
#             self._stop_event.set()
#             self._pause_event.clear()
            
#             # 等待播放线程结束
#             if self._playback_thread and self._playback_thread.is_alive():
#                 self._playback_thread.join(timeout=2.0)
            
#             self._set_state(PlaybackState.STOPPED)
#             self._reset_stats()
#             logger.info("播放已停止")
            
#             return True

#     def seek(self, time_offset: float) -> bool:
#         """
#         跳转到指定时间位置
        
#         Args:
#             time_offset: 时间偏移(秒)，相对于bag开始时间
            
#         Returns:
#             True 跳转成功；False 失败
#         """
#         if self._bag_info is None:
#             return False
        
#         # 限制在有效范围内
#         time_offset = max(0.0, min(self._bag_info.duration, time_offset))
        
#         with self._lock:
#             if self._state == PlaybackState.PLAYING:
#                 # 如果正在播放，需要重新启动
#                 was_playing = True
#                 self.stop()
#             else:
#                 was_playing = False
            
#             self._start_offset = time_offset
            
#             if was_playing:
#                 self.play(rate=self._playback_rate, 
#                          loop=self._loop_playback,
#                          start_offset=self._start_offset,
#                          duration=self._duration_limit,
#                          topics=self._topic_filter)
        
#         logger.info(f"跳转到时间位置: {time_offset:.2f}s")
#         return True

#     def set_rate(self, rate: float) -> bool:
#         """
#         设置播放速率
        
#         Args:
#             rate: 播放速率倍数
            
#         Returns:
#             True 设置成功；False 失败
#         """
#         rate = max(0.1, min(10.0, rate))
        
#         with self._lock:
#             self._playback_rate = rate
#             self._stats.playback_rate = rate
        
#         logger.info(f"设置播放速率: {rate}x")
#         return True

#     def get_state(self) -> PlaybackState:
#         """获取当前播放状态"""
#         return self._state

#     def get_bag_info(self) -> Optional[BagInfo]:
#         """获取当前bag文件信息"""
#         return self._bag_info

#     def get_stats(self) -> PlaybackStats:
#         """获取播放统计信息"""
#         with self._lock:
#             return PlaybackStats(
#                 current_time=self._stats.current_time,
#                 progress_percent=self._stats.progress_percent,
#                 messages_played=self._stats.messages_played,
#                 elapsed_time=self._stats.elapsed_time,
#                 playback_rate=self._stats.playback_rate
#             )

#     def add_state_callback(self, callback: Callable[[PlaybackState], None]) -> None:
#         """添加状态变化回调函数"""
#         if callback not in self._state_callbacks:
#             self._state_callbacks.append(callback)

#     def remove_state_callback(self, callback: Callable[[PlaybackState], None]) -> None:
#         """移除状态变化回调函数"""
#         if callback in self._state_callbacks:
#             self._state_callbacks.remove(callback)

#     def add_progress_callback(self, callback: Callable[[PlaybackStats], None]) -> None:
#         """添加进度更新回调函数"""
#         if callback not in self._progress_callbacks:
#             self._progress_callbacks.append(callback)

#     def remove_progress_callback(self, callback: Callable[[PlaybackStats], None]) -> None:
#         """移除进度更新回调函数"""
#         if callback in self._progress_callbacks:
#             self._progress_callbacks.remove(callback)

#     def _playback_worker(self) -> None:
#         """播放工作线程"""
#         try:
#             while not self._stop_event.is_set():
#                 self._play_once()
                
#                 if not self._loop_playback:
#                     break
                    
#                 if self._stop_event.is_set():
#                     break
                
#                 logger.info("循环播放：重新开始")
                
#         except Exception as e:
#             logger.error(f"播放过程出错: {e}")
#             self._set_state(PlaybackState.ERROR)
#             return
        
#         if not self._stop_event.is_set():
#             self._set_state(PlaybackState.FINISHED)
#         else:
#             self._set_state(PlaybackState.STOPPED)

#     def _play_once(self) -> None:
#         """播放一次完整的bag"""
#         if not self._bag_file or not self._bag_info:
#             return
        
#         self._start_timestamp = time.time()
#         bag_start_time = None
#         playback_start_time = None
        
#         try:
#             with Bag(self._bag_file, 'r') as bag:
#                 # 计算实际的开始和结束时间
#                 actual_start = self._bag_info.start_time + self._start_offset
#                 actual_end = self._bag_info.end_time
                
#                 if self._duration_limit:
#                     actual_end = min(actual_end, actual_start + self._duration_limit)
                
#                 message_count = 0
                
#                 for topic, msg, bag_time in bag.read_messages(
#                     start_time=rospy.Time.from_sec(actual_start),
#                     end_time=rospy.Time.from_sec(actual_end),
#                     topics=self._topic_filter
#                 ):
#                     # 检查停止信号
#                     if self._stop_event.is_set():
#                         break
                    
#                     # 检查暂停状态
#                     while self._pause_event.is_set() and not self._stop_event.is_set():
#                         time.sleep(0.1)
                    
#                     if self._stop_event.is_set():
#                         break
                    
#                     # 初始化计时
#                     if bag_start_time is None:
#                         bag_start_time = bag_time.to_sec()
#                         playback_start_time = time.time()
                    
#                     # 计算应该等待的时间
#                     bag_elapsed = bag_time.to_sec() - bag_start_time
#                     real_elapsed = time.time() - playback_start_time
#                     expected_elapsed = bag_elapsed / self._playback_rate
                    
#                     sleep_time = expected_elapsed - real_elapsed
#                     if sleep_time > 0:
#                         time.sleep(sleep_time)
                    
#                     # 发布消息（这里可以根据需要实现消息转发）
#                     # 注意：这里简化处理，实际应用中可能需要重新发布到相应话题
                    
#                     message_count += 1
                    
#                     # 更新统计信息
#                     self._update_stats(bag_time.to_sec() - self._bag_info.start_time, message_count)
                    
#                     # 定期更新进度
#                     if message_count % 100 == 0:
#                         self._notify_progress()
                
#         except Exception as e:
#             logger.error(f"播放bag文件出错: {e}")
#             raise

#     def _update_stats(self, current_time: float, message_count: int) -> None:
#         """更新统计信息"""
#         with self._lock:
#             self._stats.current_time = current_time
#             self._stats.messages_played = message_count
#             self._stats.playback_rate = self._playback_rate
            
#             if self._bag_info:
#                 self._stats.progress_percent = (current_time / self._bag_info.duration) * 100
            
#             if self._start_timestamp:
#                 self._stats.elapsed_time = time.time() - self._start_timestamp

#     def _notify_progress(self) -> None:
#         """通知进度更新"""
#         stats = self.get_stats()
#         for callback in self._progress_callbacks:
#             try:
#                 callback(stats)
#             except Exception as e:
#                 logger.error(f"进度回调执行失败: {e}")

#     def _set_state(self, new_state: PlaybackState) -> None:
#         """设置播放状态并通知回调"""
#         with self._lock:
#             if self._state != new_state:
#                 old_state = self._state
#                 self._state = new_state
#                 logger.debug(f"状态变化: {old_state.value} -> {new_state.value}")
                
#                 # 通知状态回调
#                 for callback in self._state_callbacks:
#                     try:
#                         callback(new_state)
#                     except Exception as e:
#                         logger.error(f"状态回调执行失败: {e}")

#     def _reset_stats(self) -> None:
#         """重置统计信息"""
#         with self._lock:
#             self._stats = PlaybackStats(playback_rate=self._playback_rate)
#             self._start_timestamp = None


# # —— 全局便捷调用 —— #
# _global_bag_player: Optional[BagPlayer] = None


# def get_bag_player() -> BagPlayer:
#     """
#     单例模式获取全局 BagPlayer 实例
#     """
#     global _global_bag_player
#     if _global_bag_player is None:
#         _global_bag_player = BagPlayer()
#     return _global_bag_player


# def play_bag(bag_file: str, 
#              rate: float = 1.0,
#              loop: bool = False,
#              topics: Optional[List[str]] = None) -> bool:
#     """
#     全局便捷函数：播放指定的bag文件
    
#     Args:
#         bag_file: bag文件路径
#         rate: 播放速率
#         loop: 是否循环播放
#         topics: 要播放的话题列表
        
#     Returns:
#         True 播放成功；False 失败
#     """
#     player = get_bag_player()
    
#     if not player.load_bag(bag_file):
#         return False
    
#     return player.play(rate=rate, loop=loop, topics=topics)


# def get_bag_info(bag_file: str) -> Optional[BagInfo]:
#     """
#     全局便捷函数：获取bag文件信息
    
#     Args:
#         bag_file: bag文件路径
        
#     Returns:
#         BagInfo对象或None
#     """
#     player = BagPlayer()  # 临时实例
#     if player.load_bag(bag_file):
#         return player.get_bag_info()
#     return None


# def stop_bag_playback() -> bool:
#     """
#     全局便捷函数：停止当前播放
#     """
#     player = get_bag_player()
#     return player.stop()