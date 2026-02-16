from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = "mil_robogym"
projects_root = Path("projects")

project_data_files = []
if projects_root.exists():
    for path in projects_root.rglob("*"):
        if path.is_file():
            relative_parent = path.relative_to(projects_root).parent
            install_dir = Path("share") / package_name / "projects" / relative_parent
            project_data_files.append((str(install_dir), [str(path)]))

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        *project_data_files,
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="daniel-parra",
    maintainer_email="daniel27parra@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "mil_robogym = mil_robogym.main:main",
        ],
    },
)
