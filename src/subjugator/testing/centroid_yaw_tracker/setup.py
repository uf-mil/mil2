from setuptools import find_packages, setup

package_name = "centroid_yaw_tracker"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jh",
    maintainer_email="nottellingu@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "centroid_yaw_tracker = centroid_yaw_tracker.centroid_yaw_tracker_node:main",
        ],
    },
)
