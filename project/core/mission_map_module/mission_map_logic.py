"""Core logic for coverage marking and path tracking."""

from __future__ import annotations

from typing import List, Tuple

import numpy as np


class MapLogic:
    """Coordinate transform and cell marking."""

    def __init__(self, resolution: float, origin: Tuple[float, float, float]) -> None:
        self.resolution = resolution
        self.origin = origin
    
    def world_to_pixel(self, x: float, y: float) -> Tuple[int, int]:
        """将机器人在世界坐标系中的 (x,y) → 栅格像素 (px,py)。"""
        px = int((x - self.origin[0]) / self.resolution)
        py = int((y - self.origin[1]) / self.resolution)
        return px, py

    def mark_cell(self, covered: np.ndarray, cell: Tuple[int, int]) -> None:
        """在 `covered` 矩阵中,将指定像素点标记为覆盖(255)"""
        x, y = cell
        if 0 <= y < covered.shape[0] and 0 <= x < covered.shape[1]:
            covered[y, x] = 255

    def add_path(self, path: List[Tuple[int, int]], cell: Tuple[int, int]) -> None:
        """将当前像素点追加到路径列表，用于后续绘制轨迹。"""
        path.append(cell)
