from glob import glob

from setuptools import find_packages, setup

package_name = "prop_missions"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UF MIL",
    maintainer_email="software@mil.ufl.edu",
    description="The prop boat's RobotX missions: Safe Passage and docking.",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "docking_mission = prop_missions.docking_mission:main",
            "fake_field = prop_missions.fake_field:main",
            "mock_ocs = prop_missions.mock_ocs:main",
            "safe_passage = prop_missions.safe_passage:main",
            "survey = prop_missions.survey:main",
        ],
    },
)
