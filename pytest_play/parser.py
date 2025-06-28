"""YAML parser for pytest-play."""

from __future__ import annotations

import yaml
from dataclasses import dataclass
from pathlib import Path
from typing import List, Dict, Any


@dataclass
class TestCase:
    name: str
    steps: List[Dict[str, Any]]
    assertions: List[Dict[str, Any]]


class Parser:
    def __init__(self, base: Path):
        self.base = Path(base)

    def collect_all(self) -> List[TestCase]:
        cases: List[TestCase] = []
        for p in sorted(self.base.glob("*.yaml")):
            cases.extend(self.parse_file(p))
        return cases

    def parse_file(self, path: Path) -> List[TestCase]:
        content = yaml.safe_load(path.read_text("utf-8")) or {}
        if isinstance(content, list):
            datas = content
        else:
            datas = [content]
        cases: List[TestCase] = []
        for data in datas:
            name = data.get("test_name", path.stem)
            steps = data.get("steps", [])
            assertions = data.get("assertions", [])
            cases.append(TestCase(name=name, steps=steps, assertions=assertions))
        return cases


def collect_all(base: Path) -> List[TestCase]:
    return Parser(base).collect_all()
