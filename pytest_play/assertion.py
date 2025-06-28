from __future__ import annotations

import time
from typing import Any, Dict, Iterable


DEFAULT_TIMEOUT = 30
DEFAULT_INTERVAL = 0.5


def wait_until(cond, timeout=DEFAULT_TIMEOUT, interval=DEFAULT_INTERVAL):
    end_time = time.time() + timeout
    while time.time() < end_time:
        if cond():
            return True
        time.sleep(interval)
    return False


def _get_attr(obj: Any, field: str):
    for part in field.split('.'):
        obj = getattr(obj, part)
    return obj


def check(assertions: Iterable[dict], ctx: Dict[str, Any]) -> None:
    for a in assertions:
        if a.get("type") == "state":
            field = a["field"]
            value = a["value"]
            timeout = a.get("timeout", DEFAULT_TIMEOUT)
            if not wait_until(lambda: _get_attr(ctx["robot_state"], field).name == value, timeout):
                current = _get_attr(ctx["robot_state"], field).name
                raise AssertionError(f"{field} expected {value}, got {current}")
        else:
            raise ValueError(f"unsupported assertion type: {a}")
