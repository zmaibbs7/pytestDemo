"""pytest plugin hooks for pytest-play."""
from __future__ import annotations

import pytest
from pathlib import Path

from . import parser, executor, assertion


def pytest_addoption(parser):
    parser.addoption("--yd", action="store", default="yaml_test", help="yaml dir")
    parser.addoption("--env", action="store", default="local", help="env")
    parser.addoption("--file", action="store", default="", help="yaml file")


def pytest_generate_tests(metafunc):
    if "case" not in metafunc.fixturenames:
        return
    yd = Path(metafunc.config.getoption("--yd"))
    file = metafunc.config.getoption("--file")
    if file:
        paths = [yd / file]
    else:
        paths = list(yd.glob("*.yaml"))
    cases = []
    for p in paths:
        cases.extend(parser.Parser(p.parent).parse_file(p))
    ids = [c.name for c in cases]
    metafunc.parametrize("case", cases, ids=ids)


@pytest.fixture
def ctx():
    return executor.ExecutionContext()


def test_yaml_case(case, ctx):
    executor.run_steps(case.steps, ctx)
    assertion.check(case.assertions, ctx)
