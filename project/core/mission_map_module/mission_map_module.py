"""MissionMap module: manage map data, region masks and control interfaces."""

from __future__ import annotations

import json
import os
from dataclasses import dataclass
from typing import Dict, List, Tuple, Optional

import numpy as np
from PIL import Image

from project.utils.logging import log

from .mission_map_logic import MapLogic
from .mission_map_output import MapOutput
from .mission_map_parser import MapParser

@dataclass
class MapMeta:
    """Store map resolution and origin."""

    resolution: float
    origin: Tuple[float, float, float]
    size: Tuple[int, int]

    def __str__(self) -> str:
        return (
            f"MapMeta(resolution={self.resolution}, "
            f"origin=({self.origin[0]}, {self.origin[1]}), "
            f"size=({self.size[0]}, {self.size[1]}))"
        )


class MissionMap:
    """Load map, maintain data, region masks and control recording."""

    def __init__(self) -> None:
        # 基础地图
        self.map_meta: Optional[MapMeta] = None
        self.origin_map: Optional[np.ndarray] = None

        # 动态覆盖 / 轨迹
        self.covered: Optional[np.ndarray] = None
        self.path: List[Tuple[int, int]] = []

        # 地图要素掩膜（region / forbid 等）
        self.regions: Dict[int, np.ndarray] = {}
        self.task_regions: List[int] = []

        # 工具类
        self.logic: Optional[MapLogic] = None
        self.output = MapOutput()

        # 运行状态
        self.running = False

    # ------------------------------------------------------------------ #
    # Initialization & control                                           #
    # ------------------------------------------------------------------ #
    def load_map(self, map_path: str) -> None:
        """载入底图与元数据，并解析地图要素。"""

        # 0. 构造路径
        nav_dir = os.path.join(map_path, "navigation")
        pgm_path = os.path.join(nav_dir, "map.pgm")
        json_path = os.path.join(nav_dir, "map_limits.json")
        log.info("Loading map from %s and %s", pgm_path, json_path)

        # 1. 读取底图
        with Image.open(pgm_path) as pgm:
            self.origin_map = np.array(pgm)

        # 2. 读取元数据
        with open(json_path, "r", encoding="utf-8") as f:
            info = json.load(f)

        resolution = float(info.get("resolution", 0.05))
        origin = (
            float(info.get("map_origin_x", 0.0)),
            float(info.get("map_origin_y", 0.0)),
            0.0,
        )
        size = (
            int(info.get("num_x_cells", self.origin_map.shape[1])),
            int(info.get("num_y_cells", self.origin_map.shape[0])),
        )

        # 3. 基本初始化
        self.map_meta = MapMeta(resolution=resolution, origin=origin, size=size)
        self.covered = np.zeros(self.origin_map.shape, dtype=np.uint8)
        self.path = []
        self.logic = MapLogic(resolution, origin)
        log.info("Map loaded: size=%s map_meta=%s",self.map_meta.size,self.map_meta)

        # 4. 解析地图要素（region / no-go / island …）
        try:
            parser = MapParser(self.map_meta)
            self.regions = parser.parse(nav_dir)
        except FileNotFoundError:
            # 若缺少 config.json 则跳过要素解析
            log.warning("No config.json found in %s; skip element parsing", nav_dir)
        self.task_regions = list(self.regions.keys())

        # 5. test: 导出解析后的区域掩膜为 PGM，验证是否正确
        export_dir = os.path.join(nav_dir, "test_masks")
        os.makedirs(export_dir, exist_ok=True)
        for rid, mask in self.regions.items():
            mask_img = Image.fromarray(mask)
            mask_path = os.path.join(export_dir, f"region_{rid}.pgm")
            mask_img.save(mask_path)
            log.info("Exported region %d mask to %s", rid, mask_path)

    # ------------------------------------------------------------------ #
    # 控制接口                                                            #
    # ------------------------------------------------------------------ #
    def set_task_regions(self, region_ids: List[int] | None) -> None:
        """指定本次任务需要清扫的 *region id* 列表。 传入 ``None`` 或空列表表示 **全部区域**。"""
        if not region_ids:
            self.task_regions = list(self.regions.keys())
        else:
            self.task_regions = [rid for rid in region_ids if rid in self.regions]
        log.info("Task regions set to: %s", self.task_regions)

    def start(self) -> None:
        if not self.map_meta:
            raise RuntimeError("Map not loaded")
        self.covered = np.zeros_like(self.origin_map)
        self.path = []
        self.running = True
        log.info("Recording started")

    def pause(self) -> None:
        if self.running:
            self.running = False
            log.info("Recording paused")

    def resume(self) -> None:
        if not self.running:
            self.running = True
            log.info("Recording resumed")

    def stop(self) -> None:
        """Stop recording without saving results."""
        self.running = False
        log.info("Recording stopped")

    # ---- export result ----
    def _build_expected_mask(self) -> np.ndarray:
        """根据 `self.task_regions` 合并生成期望清扫掩膜。"""
        if not self.regions:
            return np.zeros_like(self.origin_map, dtype=np.uint8)
        expected = np.zeros_like(self.origin_map, dtype=np.uint8)
        for rid in self.task_regions:
            mask = self.regions.get(rid)
            if mask is not None:
                expected = np.maximum(expected, mask)
        return expected

    def export_results(self, output_dir: str) -> None:
        """输出实际、期望及合成地图。"""
        if (
            self.origin_map is None
            or self.covered is None
            or self.map_meta is None
        ):
            raise RuntimeError("Map not loaded")

        os.makedirs(output_dir, exist_ok=True)

        # 1. 实际清扫
        self.output.save_mask_map(
            os.path.join(output_dir, "sim_real_mowing.pgm"),
            self.covered,
        )

        # 2. 期望清扫
        expected = self._build_expected_mask()
        self.output.save_expected_map(
            os.path.join(output_dir, "sim_expect_mowing.pgm"),
            expected,
        )

        # 3. 机器人路径
        self.output.save_path_map(
            os.path.join(output_dir, "sim_robot_path.pgm"),
            self.path,
            self.map_meta.size,
        )

        # 4. 合成图：base → expected(gray) → real(green) → path(red)
        self.output.save_combined_map(
            os.path.join(output_dir, "combined_map.jpg"),
            self.origin_map,
            expected,
            self.covered,
            self.path,
        )

        # 5. 统计报告
        self.output.save_report(
            os.path.join(output_dir, "coverage_report.json"),
            self.get_stats(),
        )

    # ---- Pose update ----
    def update_pose(self, x: float, y: float, theta: float) -> None:
        if not self.running or not self.logic or self.covered is None:
            return
        pixel = self.logic.world_to_pixel(x, y)
        self.logic.mark_cell(self.covered, pixel)
        self.logic.add_path(self.path, pixel)

    # ---- Data access ----
    def get_stats(self) -> Dict[str, float]:
        """返回清扫完成百分比。

        - *expected*  : 由 ``self.task_regions`` 合成的期望清扫掩膜
        - *work_done* : 机器人实际经过且位于期望区域内的栅格
        """
        if self.covered is None or self.origin_map is None:
            return {}

        # 1) 期望清扫区域
        expected = self._build_expected_mask()
        expected_cnt = np.count_nonzero(expected)
        if expected_cnt == 0:
            return {"finished": 0.0}  # 没有期望区域时直接返回 0

        # 2) 实际清扫 ∩ 期望区域
        work_covered = np.logical_and(self.covered > 0, expected > 0)
        work_cnt = np.count_nonzero(work_covered)

        # 3) 完成比例
        finished = round(work_cnt / expected_cnt * 100, 2)
        return {"finished": finished}

    def get_map(self) -> Tuple[np.ndarray, np.ndarray]:
        if self.origin_map is None or self.covered is None:
            raise RuntimeError("Map not loaded")
        return self.origin_map.copy(), self.covered.copy()

    def get_path(self) -> List[Tuple[int, int]]:
        return list(self.path)

    def get_region_mask(self, region_id: int) -> Optional[np.ndarray]:
        """获取指定区域掩膜，若无则返回 None。"""
        return self.regions.get(region_id)
