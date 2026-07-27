"""The CLI's exit-code discipline is load-bearing, so it gets its own tests.

Exit 1 means the mission failed; anything the harness itself rejects must be 2.
"""

import dataclasses

import pytest
from subjugator_bringup.task_runner import cli, tasks
from subjugator_bringup.task_runner.exitcodes import EXIT_HARNESS, EXIT_OK


def parse(*argv):
    return cli.build_parser().parse_args(list(argv))


def test_list_exits_ok_and_prints_every_stage(capsys):
    assert cli.main(["--list"]) == EXIT_OK
    out = capsys.readouterr().out
    for name, stage in tasks.TASKS[5].stages.items():
        assert name in out
        assert stage.mission in out
    assert "(default)" in out


def test_listing_flags_a_hand_set_budget(monkeypatch):
    """A budget_note marks an unverified budget, so --list has to surface it."""
    spec = tasks.TASKS[5]
    noted = dataclasses.replace(
        spec.stages["calib"],
        budget_note="hand-set, not derived",
    )
    patched = dataclasses.replace(spec, stages={**spec.stages, "calib": noted})
    monkeypatch.setattr(tasks, "TASKS", {5: patched})

    assert "hand-set, not derived" in cli.format_listing()


def test_unknown_task_exits_harness_and_prints_the_listing(capsys):
    with pytest.raises(SystemExit) as exc:
        cli.resolve(parse("99"))
    assert exc.value.code == EXIT_HARNESS
    err = capsys.readouterr().err
    assert "not wired" in err
    assert "task 5" in err


def test_unknown_stage_exits_harness(capsys):
    with pytest.raises(SystemExit) as exc:
        cli.resolve(parse("5", "--stage", "nope"))
    assert exc.value.code == EXIT_HARNESS
    assert "unknown stage" in capsys.readouterr().err


def test_unknown_start_exits_harness(capsys):
    with pytest.raises(SystemExit) as exc:
        cli.resolve(parse("5", "--start", "nope"))
    assert exc.value.code == EXIT_HARNESS
    assert "unknown start" in capsys.readouterr().err


def test_missing_task_number_exits_harness():
    assert cli.main([]) == EXIT_HARNESS


def test_resolve_returns_the_stage_and_its_start():
    spec, stage_name, stage, start_name = cli.resolve(parse("5", "--stage", "calib"))
    assert spec is tasks.TASKS[5]
    assert stage_name == "calib"
    assert stage is spec.stages["calib"]
    assert start_name == "over_table"


def test_stage_default_comes_from_the_task():
    _, stage_name, _, _ = cli.resolve(parse("5"))
    assert stage_name == tasks.TASKS[5].default_stage


def test_start_flag_overrides_the_stage_start():
    _, _, _, start_name = cli.resolve(parse("5", "--stage", "calib", "--start", "near"))
    assert start_name == "near"


def test_role_falls_back_to_the_task_role():
    args = parse("5")
    cli.resolve(args)
    assert args.role == tasks.TASKS[5].role


def test_role_flag_wins_over_the_task_role():
    args = parse("5", "--role", "mapping")
    cli.resolve(args)
    assert args.role == "mapping"
