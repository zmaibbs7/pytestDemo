from pytest_play.registry import step, get_action


def test_step_register_and_call():
    calls = []

    @step("hello")
    def hello(ctx):
        calls.append("called")

    action = get_action("hello")
    action({})

    assert calls == ["called"]
