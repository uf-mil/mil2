"""Tests for demo folder creation and duplicate handling."""

from pathlib import Path

import pytest
import yaml

from mil_robogym.data_collection.filesystem import (
    append_demo_action_row,
    append_demo_metadata_entry,
    append_demo_numerical_row,
    create_demo_folder,
    get_demo_topic_data_dir,
    initialize_demo_numerical_csv_headers,
    write_demo_topic_payload_file,
    write_demo_topic_png,
)


def test_create_demo_folder_creates_files(tmp_path: Path):
    """Creates a demo folder and writes config + data manifest scaffolding."""
    project_dir = tmp_path / "projects" / "start_gate_agent"
    project_dir.mkdir(parents=True)

    demo_dir, _cfg = create_demo_folder(
        project_dir,
        name="Demo 1",
        sampling_rate=1.0,
        start_position=[1.0, 2.0, 3.0, 4.0],
    )

    assert demo_dir.exists()
    assert demo_dir.name == "demo_1"

    cfg = yaml.safe_load((demo_dir / "config.yaml").read_text(encoding="utf-8"))
    assert cfg["robogym_demo"]["name"] == "Demo 1"
    assert cfg["robogym_demo"]["sampling_rate"] == 1.0
    assert cfg["robogym_demo"]["start_position"] == [1.0, 2.0, 3.0, 4.0]

    metadata_csv = demo_dir / "metadata.csv"
    numerical_csv = demo_dir / "data" / "numerical" / "data.csv"
    actions_csv = demo_dir / "data" / "actions" / "data.csv"
    assert metadata_csv.is_file()
    assert numerical_csv.is_file()
    assert actions_csv.is_file()
    assert metadata_csv.read_text(encoding="utf-8") == (
        "topic,relative_path\n"
        "/numerical,data/numerical/data.csv\n"
        "/actions,data/actions/data.csv\n"
    )


def test_create_demo_folder_raises_if_exists(tmp_path: Path):
    """Raises when creating a demo folder that already exists."""
    project_dir = tmp_path / "projects" / "demo_project"
    project_dir.mkdir(parents=True)

    create_demo_folder(
        project_dir,
        name="Demo 1",
        sampling_rate=1.0,
    )

    with pytest.raises(FileExistsError):
        create_demo_folder(
            project_dir,
            name="Demo 1",
            sampling_rate=1.0,
        )


def test_topic_data_folder_and_metadata_entry(tmp_path: Path):
    """Creates topic data folders and appends file references to metadata.csv."""
    project_dir = tmp_path / "projects" / "demo_project"
    project_dir.mkdir(parents=True)
    demo_dir, _cfg = create_demo_folder(
        project_dir,
        name="Demo 1",
        sampling_rate=1.0,
    )

    topic_dir = get_demo_topic_data_dir(demo_dir, topic="/frontcam/image_raw")
    assert topic_dir.name == "frontcam_image_raw"

    image_path = write_demo_topic_png(
        demo_dir,
        topic="/frontcam/image_raw",
        file_name="image0.png",
        png_data=b"\x89PNG\r\n\x1a\n",
    )
    assert image_path.is_file()
    assert image_path.name == "image0.png"

    initialize_demo_numerical_csv_headers(
        demo_dir,
        ["/imu/data:orientation.x", "/imu/data:orientation.y", "/trajectory/4_deg:yaw"],
    )
    append_demo_numerical_row(demo_dir, [0, 1.5, -2.0])
    numerical_lines = (
        (demo_dir / "data" / "numerical" / "data.csv")
        .read_text(encoding="utf-8")
        .splitlines()
    )
    assert numerical_lines == [
        "/imu/data:orientation.x,/imu/data:orientation.y,/trajectory/4_deg:yaw",
        "0,1.5,-2.0",
    ]

    append_demo_action_row(demo_dir, [0.1, -0.2, 0.0, 1.57])
    action_lines = (
        (demo_dir / "data" / "actions" / "data.csv")
        .read_text(encoding="utf-8")
        .splitlines()
    )
    assert action_lines == [
        "delta_x,delta_y,delta_z,delta_yaw",
        "0.1,-0.2,0.0,1.57",
    ]

    yolo_path = write_demo_topic_payload_file(
        demo_dir,
        topic="/yolo/detections",
        file_name="detections_0000.npy",
        payload=b"npy-bytes",
    )
    assert yolo_path.is_file()
    assert yolo_path == demo_dir / "data" / "yolo_detections" / "detections_0000.npy"
    assert yolo_path.read_bytes() == b"npy-bytes"

    append_demo_metadata_entry(
        demo_dir,
        topic="/frontcam/image_raw",
        relative_path="data/frontcam_image_raw/image1.png",
    )
    metadata_lines = (
        (demo_dir / "metadata.csv").read_text(encoding="utf-8").splitlines()
    )
    assert metadata_lines[0] == "topic,relative_path"
    assert metadata_lines[1] == "/numerical,data/numerical/data.csv"
    assert metadata_lines[2] == "/actions,data/actions/data.csv"
    assert metadata_lines[3] == "/frontcam/image_raw,data/frontcam_image_raw/image0.png"
    assert (
        metadata_lines[4] == "/yolo/detections,data/yolo_detections/detections_0000.npy"
    )
    assert metadata_lines[5] == "/frontcam/image_raw,data/frontcam_image_raw/image1.png"
