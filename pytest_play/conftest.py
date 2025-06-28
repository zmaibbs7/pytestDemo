from __future__ import annotations

from pathlib import Path
from typing import Iterator

import pytest

from .parser import collect_all, collect_from_file, TestCase
from .executor import ExecutionContext, run_steps
from .assertion import check


def pytest_addoption(parser: pytest.Parser) -> None:
    group = parser.getgroup("pytest-play")
    group.addoption("--yd", action="store", default="yaml_test", help="YAML directory")
    group.addoption("--env", action="store", default="", help="env name")
    group.addoption("--file", action="store", default="", help="specific yaml file")


def pytest_generate_tests(metafunc: pytest.Metafunc) -> None:
    if "case" in metafunc.fixturenames:
        yd = Path(metafunc.config.getoption("--yd"))
        file = metafunc.config.getoption("--file")
        if file:
            cases = [collect_from_file(yd / file)]
        else:
            cases = collect_all(yd)
        metafunc.parametrize("case", cases, ids=[c.name for c in cases])


@pytest.fixture
def ctx() -> ExecutionContext:
    return ExecutionContext(robot_state=None)


def test_yaml_case(case: TestCase, ctx: ExecutionContext) -> None:
    run_steps(case.steps, ctx)
    if case.assertions:
        check(case.assertions, ctx)
