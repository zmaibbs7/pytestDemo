from pytest_play.registry import step


@step("touch_state")
def touch_state(ctx):
    ctx["touched"] = "yes"
