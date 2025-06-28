"""Map output utilities for images and reports."""

from __future__ import annotations

import json
from typing import Dict, List, Tuple

import numpy as np
from PIL import Image, ImageDraw


class MapOutput:
    """Generate and save map images and JSON report."""
    
    def save_mask_map(self, pgm_file: str, mask: np.ndarray) -> None:
        """保存二值/灰度掩膜到 PGM。"""
        Image.fromarray(mask.astype(np.uint8)).save(pgm_file)

    def save_expected_map(self, pgm_file: str,
                          expected: np.ndarray) -> None:
        """期望清扫掩膜作为单独 PGM (255=应清扫)。"""
        Image.fromarray(expected).save(pgm_file)

    def save_path_map(self, pgm_file: str,
                      path: List[Tuple[int, int]],
                      size: Tuple[int, int]) -> None:
        """生成机器人路径图像"""
        img = Image.new("L", size, 0)
        draw = ImageDraw.Draw(img)
        if path:
            if len(path) > 1:
                draw.line(path, fill=255, width=1)
            else:
                draw.point(path[0], fill=255)
        img.save(pgm_file)

    def save_combined_map(self, jpg_file: str,
                          base_map: np.ndarray,
                          expected: np.ndarray,
                          covered: np.ndarray,
                          path: List[Tuple[int, int]]) -> None:
        # 1. 底图转 RGB
        base_rgb = Image.fromarray(base_map).convert("RGB")

        # 2. 期望区域 (浅蓝色)
        exp_mask = Image.fromarray(expected)
        base_rgb.paste((51, 178, 255), mask=exp_mask)

        # 3. 实际覆盖 (绿色)
        cov_mask = Image.fromarray(covered)
        base_rgb.paste((0, 255, 0), mask=cov_mask)

        # 4. 轨迹 (红线)
        draw = ImageDraw.Draw(base_rgb)
        if path:
            if len(path) > 1:
                draw.line(path, fill=(255, 0, 0), width=2)
            else:
                draw.point(path[0], fill=(255, 0, 0))

        # 5. 保存
        base_rgb.save(jpg_file)

    def save_report(self, json_file: str,
                    stats: Dict[str, float]) -> None:
        """保存统计信息到JSON文件"""
        with open(json_file, "w", encoding="utf-8") as f:
            json.dump(stats, f, indent=2)
