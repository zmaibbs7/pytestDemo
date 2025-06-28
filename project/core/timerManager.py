# 
import threading
import time

class TimeManager:
    """
    单线程调度器，负责：
      1) 周期性 Tick 回调
      2) 一次性超时任务
    支持 clear() 与 shutdown()，便于测试隔离与资源释放。
    """
    _instance = None
    _lock = threading.Lock()

    def __init__(self, tick_interval: float = 1.0):
        self.tick_interval = tick_interval
        self._tick_cbs: list[callable] = []
        self._timeout_tasks: list[tuple[float, int, callable]] = []
        self._mutex = threading.Lock()
        self._running = True

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    @classmethod
    def instance(cls):
        with cls._lock:
            if cls._instance is None:
                cls._instance = TimeManager()
            return cls._instance

    def clear(self):
        """
        清空所有注册的周期回调和超时任务。
        用于测试前后环境隔离。
        """
        with self._mutex:
            self._tick_cbs.clear()
            self._timeout_tasks.clear()

    def shutdown(self):
        """
        停止后台线程。测试会话结束时调用，确保无残留线程。
        """
        self._running = False
        self._thread.join()

    def addTickCallback(self, fn: callable):
        """
        注册一个周期性回调，每个 tick_interval 调用一次。
        :param fn: 无参回调函数
        """
        with self._mutex:
            self._tick_cbs.append(fn)

    def addTimeout(self, q: int, fn: callable, timeout: float):
        """
        注册一次性超时回调：
          - q: 请求 ID，用于标识
          - fn: 超时后调用的函数
          - timeout: 延迟秒数
        """
        deadline = time.time() + timeout
        with self._mutex:
            self._timeout_tasks.append((deadline, q, fn))

    def removeTimeout(self, q: int, fn: callable):
        """
        手动移除尚未触发的超时任务。
        """
        with self._mutex:
            self._timeout_tasks = [
                t for t in self._timeout_tasks
                if not (t[1] == q and t[2] == fn)
            ]

    def _run(self):
        """
        后台线程入口，循环调度周期回调与超时检查。
        """
        while self._running:
            now = time.time()

            # 周期性回调
            with self._mutex:
                tickers = list(self._tick_cbs)
            for fn in tickers:
                try:
                    fn()
                except Exception:
                    print("Error in tick callback", fn, flush=True)

            # 超时任务
            with self._mutex:
                tasks = list(self._timeout_tasks)
            for deadline, q, fn in tasks:
                if now >= deadline:
                    try:
                        fn()
                    except Exception:
                        print("Error in timeout callback", fn, flush=True)
                    self.removeTimeout(q, fn)

            time.sleep(self.tick_interval)
