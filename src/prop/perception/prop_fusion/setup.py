import os
from glob import glob

from setuptools import find_packages, setup

package_name = "prop_fusion"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Carlos Chavez",
    maintainer_email="c.chavez@ufl.edu",
    description="Places the sensors on the hull, labels the lidar's tracks with "
    "what the cameras see, and keeps those labels on the evidence grid.",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
    entry_points={
        "console_scripts": [
            "sensor_tf = prop_fusion.sensor_tf_node:main",
            "fusion = prop_fusion.fusion_node:main",
            "object_map = prop_fusion.object_map_node:main",
        ],
    },
)
