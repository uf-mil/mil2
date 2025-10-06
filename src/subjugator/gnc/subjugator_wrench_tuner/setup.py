from setuptools import find_packages, setup

package_name = "subjugator_wrench_tuner"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/wrench_tuner_launch.py"]),
        ("share/" + package_name + "/config", ["config/wrench_tuner_params.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="adamm",
    maintainer_email="amcaleer1127@gmail.com",
    description="TODO: Package description",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wrench_tuner = subjugator_wrench_tuner.wrench_tuner:main",
        ],
    },
)
