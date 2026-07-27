from subjugator_bringup.task_runner import tasks


def test_task5_is_wired():
    spec = tasks.TASKS[5]
    assert spec.default_stage in spec.stages
    assert set(spec.stages) == {"calib", "combined", "grasp", "full"}


def test_every_stage_start_exists():
    for num, spec in tasks.TASKS.items():
        for name, stage in spec.stages.items():
            assert stage.start in spec.starts, f"task {num} stage {name}"


def test_every_budget_is_positive():
    for spec in tasks.TASKS.values():
        for name, stage in spec.stages.items():
            assert stage.sim_budget > 0, name


def test_tuning_stages_start_over_the_table():
    stages = tasks.TASKS[5].stages
    assert stages["calib"].start == "over_table"
    assert stages["combined"].start == "over_table"
    assert stages["grasp"].start == "over_table"
    assert stages["full"].start == "near"


def test_start_resolves_to_world_coordinates():
    spec = tasks.TASKS[5]
    x, y, z = spec.world_start("over_table")
    assert (round(x, 2), round(y, 2), z) == (-7.70, 13.97, -0.35)
    x, y, z = spec.world_start("near")
    assert (round(x, 2), round(y, 2), z) == (-7.25, 12.50, -0.35)


def test_probes_cover_detections_and_model_poses():
    kinds = {p.kind for p in tasks.TASKS[5].probes}
    assert kinds == {"detections", "model_pose"}
