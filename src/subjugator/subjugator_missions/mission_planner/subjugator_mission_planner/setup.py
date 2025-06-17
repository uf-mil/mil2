from setuptools import find_packages, setup

package_name = "subjugator_mission_planner"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/subjugator_mission_planner"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/action",
            [
                f"{package_name}/action/NavigateAround.action",
                f"{package_name}/action/NavigateThrough.action",
                f"{package_name}/action/SearchObject.action",
            ],
        ),
    ],
    install_requires=["setuptools"],
    python_requires=">=3.8",
    zip_safe=True,
    maintainer="Adam McAleer",
    maintainer_email="amcaleer1127@gmail.com",
    description="Mission planner package with ROS2 action servers",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_planner = subjugator_mission_planner.mission_planner:main",
            "navigate_around_server = subjugator_mission_planner.navigate_around_server:main",
            "navigate_through_server = subjugator_mission_planner.navigate_through_server:main",
            "search_server = subjugator_mission_planner.search_server:main",
        ],
    },
)
