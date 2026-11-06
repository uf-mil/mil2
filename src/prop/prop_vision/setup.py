from setuptools import find_packages, setup

package_name = "prop_vision"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="UF MIL",
    maintainer_email="software@mil.ufl.edu",
    description="Vision and sensor-fusion helpers for the Prop vehicle.",
    license="TODO: License declaration",
    extras_require={"test": ["pytest"]},
)
