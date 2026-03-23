from setuptools import find_packages, setup

package_name = "subjugator_sim_actuator_client"

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
    maintainer="Carlos Chavez",
    maintainer_email="c.chavez@ufl.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sim_actuator_client = subjugator_sim_actuator_client.main:main",
        ],
    },
)
