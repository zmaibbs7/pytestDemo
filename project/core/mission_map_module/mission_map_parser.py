"""地图要素解析统一入口。

本文件提供三个核心概念：

* **`MapElement`**   —— 抽象基类，定义所有地图要素解析器必须实现的接口；
* **`RegionElement`** —— 目前唯一实现，负责解析 `type == "region"` 的区域多边形；
* **`MapParser`**     —— 高层封装，根据 *config.json* 自动分派到对应的具体解析器。

当未来需要支持诸如 *no-go zone*、*virtual wall* 等新要素时，仅需：

1. 创建继承自 `MapElement` 的新类，声明 `TYPE` 常量；
2. 实现 `load_raw()` / `build_mask()` 逻辑；
3. 在 `MapParser.ELEMENT_CLASSES` 中注册即可，无需改动其他代码。
"""

from __future__ import annotations

import json
import os
from abc import ABC, abstractmethod
from typing import Dict, List, Tuple, Type

import numpy as np
from PIL import Image, ImageDraw

from .mission_map_logic import MapLogic
from project.utils.logging import log

# --------------------------------------------------------------------- #
# 抽象基类                                                              #
# --------------------------------------------------------------------- #
class MapElement(ABC):
    """地图要素解析器基类。

    子类必须声明 `TYPE` 常量，同时实现 :meth:`load_raw` 与 :meth:`build_mask`。"""

    TYPE: str = ""

    def __init__(self, elem_info: dict, map_meta: "MapMeta") -> None:
        self.id: int = elem_info["id"]
        self._info = elem_info
        self._meta = map_meta
        self._logic = MapLogic(map_meta.resolution, map_meta.origin)

    # ---------------------------- 接口 ---------------------------- #
    @classmethod
    def matches(cls, elem_info: dict) -> bool:  # noqa: D401
        """该解析器是否适用于 *elem_info*。默认比较 `type`。"""
        return elem_info.get("type") == cls.TYPE

    @abstractmethod
    def load_raw(self, nav_path: str) -> bool:  # noqa: D401
        """加载要素所需的原始数据。

        返回 ``True`` 表示加载成功，``False`` 表示应忽略该要素。"""

    @abstractmethod
    def build_mask(self) -> np.ndarray:  # noqa: D401
        """根据原始数据生成与底图同尺寸的二值掩膜。"""

    # ---------------------------- 辅助 ---------------------------- #
    def help(self) -> str:
        """返回要素帮助信息，可用于调试或自动文档。"""
        return f"<{self.__class__.__name__}> id={self.id} type={self.TYPE}"


# --------------------------------------------------------------------- #
# 具体实现：区域                                                         #
# --------------------------------------------------------------------- #
class RegionElement(MapElement):
    """`region` 区域要素：解析多边形并光栅化。"""

    TYPE = "region"

    def __init__(self, elem_info: dict, map_meta: "MapMeta") -> None:
        super().__init__(elem_info, map_meta)
        self._points: List[Tuple[float, float]] = []

    # ------------------------- 必须实现 -------------------------- #
    def load_raw(self, nav_path: str) -> bool:
        """读取 `<id>.txt` 多边形世界坐标。"""
        txt_path = os.path.join(nav_path, f"{self.id}.txt")
        try:
            with open(txt_path, "r", encoding="utf-8") as f:
                for line in f:
                    line = line.strip()
                    if not line:
                        continue
                    x_str, y_str = line.split()
                    self._points.append((float(x_str), float(y_str)))
        except FileNotFoundError:
            log.warning("Region %d missing polygon file: %s", self.id, txt_path)
            return False
        log.info("Loaded %d points for region %d", len(self._points), self.id)
        return bool(self._points)

    def build_mask(self) -> np.ndarray:
        width, height = self._meta.size
        img = Image.new("L", (width, height), 0)
        if len(self._points) >= 3:
            px_pts = [self._logic.world_to_pixel(x, y) for x, y in self._points]
            ImageDraw.Draw(img).polygon(px_pts, outline=255, fill=255)
        return np.array(img, dtype=np.uint8)


# --------------------------------------------------------------------- #
# 解析入口                                                               #
# --------------------------------------------------------------------- #
class MapParser:
    """统一调度各要素解析器，生成区域掩膜集合。"""

    #: 注册的解析器列表；按顺序匹配，当多解析器都能处理同一元素时取第一个
    ELEMENT_CLASSES: List[Type[MapElement]] = [RegionElement]

    def __init__(self, map_meta: "MapMeta") -> None:
        self._meta = map_meta

    # ------------------------------------------------------------------ #
    def parse(self, nav_path: str) -> Dict[int, np.ndarray]:
        """读取 *nav_path/config.json*，解析所有支持的地图要素。"""
        cfg_path = os.path.join(nav_path, "config.json")
        with open(cfg_path, "r", encoding="utf-8") as f:
            cfg = json.load(f)

        masks: Dict[int, np.ndarray] = {}
        for elem in cfg.get("map", []):
            parser_cls = self._dispatch(elem)
            if not parser_cls:
                continue  # 不支持的 type
            parser = parser_cls(elem, self._meta)
            if parser.load_raw(nav_path):
                masks[parser.id] = parser.build_mask()
        log.info("Parsed %d map element(s) from %s", len(masks), nav_path)
        return masks

    # ------------------------------------------------------------------ #
    def _dispatch(self, elem_info: dict) -> Type[MapElement] | None:
        """根据 *elem_info* 的 `type` 字段选择合适的解析器类。"""
        for cls in self.ELEMENT_CLASSES:
            if cls.matches(elem_info):
                return cls
        log.debug("No parser for element: %s", elem_info)
        return None

    # ------------------------------------------------------------------ #
    @classmethod
    def register(cls, element_cls: Type[MapElement]) -> None:  # noqa: D401
        """在运行时动态注册新的要素解析器。"""
        cls.ELEMENT_CLASSES.insert(0, element_cls)
