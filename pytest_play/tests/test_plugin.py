import pytest
from pathlib import Path

from pytest_play import parser, registry, executor, assertion


def test_parser_collect():
    cases = parser.collect_all(Path(__file__).parent / "yaml")
    assert cases
    assert cases[0].name == "example"


@registry.step("set_value")
def set_value(ctx, key, value):
    ctx[key] = value


def test_run(tmp_path):
    cases = parser.collect_all(Path(__file__).parent / "yaml")
    case = cases[0]
    ctx = executor.ExecutionContext()
    executor.run_steps(case.steps, ctx)
    assertion.check(case.assertions, ctx)
    assert ctx["foo"] == "bar"
