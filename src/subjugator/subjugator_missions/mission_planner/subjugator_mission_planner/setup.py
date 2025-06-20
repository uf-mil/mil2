from setuptools import find_packages, setup

package_name = "subjugator_mission_planner"

setup(
    name=package_name,
    include_package_data=True,
    package_data={"": ["missions/*.yaml"]},
    version="0.0.1",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/subjugator_mission_planner"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/missions", ["missions/prequal.yaml"]),
        (
            f"share/{package_name}/launch",
            ["launch/mission_planner_launch.py", "launch/task_server_launch.py"],
        ),
    ],
    install_requires=["setuptools"],
    python_requires=">=3.8",
    zip_safe=True,
    maintainer="Adam McAleer",
    maintainer_email="amcaleer1127@gmail.com",
    description="Mission planner package with ROS2 action servers",
    license="",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_planner = subjugator_mission_planner.mission_planner:main",
            "navigate_around_server = subjugator_mission_planner.navigate_around_server:main",
            "navigate_through_server = subjugator_mission_planner.navigate_through_server:main",
            "search_server = subjugator_mission_planner.search_server:main",
            "movement_server = subjugator_mission_planner.movement_server:main",
        ],
    },
)
