"""Step registry for pytest-play."""
from __future__ import annotations

from typing import Callable, Dict


_REGISTRY: Dict[str, Callable[..., None]] = {}


def step(name: str) -> Callable[[Callable[..., None]], Callable[..., None]]:
    """Decorator to register a step function."""
    def wrapper(func: Callable[..., None]) -> Callable[..., None]:
        _REGISTRY[name] = func
        return func

    return wrapper


def get(name: str) -> Callable[..., None]:
    if name not in _REGISTRY:
        raise KeyError(f"unknown step: {name}")
    return _REGISTRY[name]


def registry() -> Dict[str, Callable[..., None]]:
    return dict(_REGISTRY)
