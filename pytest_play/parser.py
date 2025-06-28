from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List

import yaml


@dataclass
class TestCase:
    name: str
    steps: List[dict]
    assertions: List[dict]
    path: Path


def collect_from_file(path: Path) -> TestCase:
    data = yaml.safe_load(path.read_text()) or {}
    name = data.get("test_name", path.stem)
    steps = data.get("steps", [])
    assertions = data.get("assertions", [])
    return TestCase(name=name, steps=steps, assertions=assertions, path=path)


def collect_all(directory: Path) -> List[TestCase]:
    cases = []
    for p in sorted(directory.glob("*.yaml")):
        cases.append(collect_from_file(p))
    return cases
