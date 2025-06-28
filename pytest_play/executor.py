from __future__ import annotations

from typing import Any, Dict, List

from .registry import get_action


class ExecutionContext(Dict[str, Any]):
    pass


def run_steps(steps: List[dict], ctx: ExecutionContext) -> None:
    for step in steps:
        if step.get("type") != "action":
            raise ValueError(f"unsupported step type: {step}")
        name = step["name"]
        action = get_action(name)
        params = step.get("params", {})
        action(ctx, **params)
