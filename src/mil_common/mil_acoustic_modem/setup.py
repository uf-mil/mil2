from setuptools import find_packages, setup

package_name = "mil_acoustic_modem"

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
    maintainer="Samuel Fernandez",
    maintainer_email="me@sfernandez.dev",
    description="Handles sending and receiving data on the acoustic modem",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "mil_acoustic_modem = mil_acoustic_modem.mil_acoustic_modem:main",
        ],
    },
)
