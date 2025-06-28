from __future__ import annotations

from functools import wraps
from typing import Callable, Dict


_ACTIONS: Dict[str, Callable] = {}


def step(name: str) -> Callable:
    """Decorator to register an action function."""

    def decorator(func: Callable) -> Callable:
        _ACTIONS[name] = func

        @wraps(func)
        def wrapper(*args, **kwargs):
            return func(*args, **kwargs)

        return wrapper

    return decorator


def get_action(name: str) -> Callable:
    if name not in _ACTIONS:
        raise KeyError(f"unknown step: {name}")
    return _ACTIONS[name]
