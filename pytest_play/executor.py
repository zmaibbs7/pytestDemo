"""Execute steps sequentially."""
from __future__ import annotations

from typing import Any, Dict

from . import registry


class ExecutionContext(dict):
    """Simple context to share data between steps."""


def run_steps(steps, ctx: ExecutionContext) -> None:
    for step in steps:
        if step.get("type") != "action":
            continue
        name = step["name"]
        func = registry.get(name)
        func(ctx, **step.get("args", {}))
