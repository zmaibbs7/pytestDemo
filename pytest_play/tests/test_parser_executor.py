from pathlib import Path

from project.core.state import RobotState, StateType, WorkingStateType

from pytest_play.parser import collect_from_file
from pytest_play.executor import ExecutionContext, run_steps
from pytest_play.assertion import check
from pytest_play.registry import step


def test_parser_and_executor(tmp_path: Path):
    yaml_content = """
    test_name: demo
    steps:
      - type: action
        name: start
    assertions:
      - type: state
        field: sub_status
        value: WORKING_STATE_RUNNING
        timeout: 1
    """
    yaml_file = tmp_path / "case.yaml"
    yaml_file.write_text(yaml_content)

    # register action
    @step("start")
    def start(ctx: ExecutionContext):
        ctx["robot_state"] = RobotState(
            status=StateType.STATE_WORK,
            sub_status=WorkingStateType.WORKING_STATE_RUNNING,
        )

    case = collect_from_file(yaml_file)
    ctx = ExecutionContext()
    run_steps(case.steps, ctx)
    check(case.assertions, ctx)
