"""Simple assertion handlers for pytest-play."""
from __future__ import annotations

import time
from typing import Any, Dict, Callable


class TimeoutError(AssertionError):
    pass


def wait_until(predicate: Callable[[], bool], timeout: float) -> None:
    start = time.time()
    while time.time() - start < timeout:
        if predicate():
            return
        time.sleep(0.1)
    raise TimeoutError(f"condition not met within {timeout}s")


def check(assertions, ctx: Dict[str, Any]) -> None:
    for a in assertions:
        if a["type"] == "state":
            field = a["field"]
            expect = a["value"]
            timeout = a.get("timeout", 5)

            def pred() -> bool:
                return ctx.get(field) == expect

            wait_until(pred, timeout)
        else:
            raise AssertionError(f"unknown assertion type: {a['type']}")
