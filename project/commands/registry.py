"""
解析 `config/command.yaml`  
- SimpleCommand: YAML→动态生成  
- ComplexCommand: import module.class  

暴露：  
    register(qual, cls)     # 供外部手动注册（可选）  
    send(qualname, …)       # 统一调用入口
"""
from __future__ import annotations

import importlib
from pathlib import Path
from types import new_class
from typing import Any, Dict, Final, Type

import yaml

from .base import Command
from project.utils.logging import log

# ---------------- 读取 YAML ----------------
_CFG_PATH: Final[Path] = Path(__file__).resolve().parent.parent / "config/command.yaml"
_CFG: Dict[str, Dict[str, Dict[str, Any]]] = yaml.safe_load(_CFG_PATH.read_text("utf-8"))

# qualname → Command subclass
_REGISTRY: Dict[str, Type[Command]] = {}

def register(qualname: str, cls: Type[Command]) -> None:
    if qualname in _REGISTRY:
        raise RuntimeError(f"duplicate command: {qualname}")
    log.info(f'register key:{qualname} = {cls}')
    _REGISTRY[qualname] = cls

# ---------------- SimpleCommand 工厂 ----------------
def _make_work(domain: str, name: str, meta: Dict[str, Any]) -> Type[Command]:
    """
    根据 working 域的 YAML，动态生成类，并注入 build_payload
    """
    def build_payload(self):
        # 构造 working 域专用的 payload 格式
        # m: method, p: priority, o: order, d: 业务参数
        return {
            # q 由 CommandManager 加，不在这里填
            "m": getattr(self, "METHOD", None),
            "p": getattr(self, "PRIORITY", None),
            "o": getattr(self, "ORDER", None),
            "d": self.params   # 业务参数由 send() 合并后传入
        }

    attrs = {
        "__doc__": f"auto simple command {domain}.{name}",
        "command_name": f"{domain}.{name}", 
        "TIMEOUT": meta.get("timeout", Command.TIMEOUT),
        "METHOD": meta.get("method"),
        "PRIORITY": meta.get("priority"),
        "ORDER": meta.get("order"),
        "build_payload": build_payload,
    }

    return new_class(f"{domain}_{name}_Cmd", (Command,), {}, lambda ns: ns.update(attrs))

def _make_mapping(domain: str, name: str, meta: Dict[str, Any]) -> Type[Command]:
    def build_payload(self):
        # 举例，mapping域要求业务参数都平铺
        payload = dict(self.params)
        payload.update({
            "cmd": name,
            "version": meta.get("version", 1)
        })
        return payload
    attrs = {
        "command_name": f"{domain}.{name}",
        "TIMEOUT": meta.get("timeout", Command.TIMEOUT),
        "build_payload": build_payload,
    }
    return new_class(f"{domain}_{name}_Cmd", (Command,), {}, lambda ns: ns.update(attrs))

# ---------------- 解析并注册 ----------------
for dom, cmds in _CFG.items():
    log.info(f'dom = {dom}')
    log.info(f'cmds = {cmds}')
    for nm, meta in cmds.items():
        qual = f"{dom}.{nm}"
        log.info(f'nm = {nm}')
        log.info(f'meta = {meta}')
        
        # 分别构造
        if dom == "working":
            cls = _make_work(dom, nm, meta)
        elif dom == "mapping":
            cls = _make_mapping(dom, nm, meta)


        register(qual, cls)


# ---------------- 统一发送 ----------------
def send(qualname: str, *, param: Dict[str, Any] | None = None, timeout: float | None = None, **extra):
    """
    send("working.mow_global", param={...}, timeout=45)
    """
    log.info("send AAAAAAAAAAAAAAAAAAAAAAAAAA")
    log.info(f'qualname = {qualname}')
    if qualname not in _REGISTRY:
        raise KeyError(f"unknown command '{qualname}'")

    merged = dict(extra)
    log.info(f'merged = {merged}')
    if param:
        merged.update(param)
    
    cmd = _REGISTRY[qualname](merged) # 初始化对象

    try:
        return cmd.send(timeout=timeout)
    except TypeError:  # 子类 send 没有 timeout 形参
        return cmd.send()
