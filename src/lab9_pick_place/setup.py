import os
from glob import glob
from setuptools import find_packages, setup

package_name = "lab9_pick_place"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lab 9 group",
    maintainer_email="student@todo.todo",
    description="Lab 9 on the real robot: vision_lab9, brain_lab9, controller_lab9",
    license="TODO",
    entry_points={
        "console_scripts": [
            "vision_lab9 = lab9_pick_place.vision_lab9:main",        # Lab PC
            "brain_lab9 = lab9_pick_place.brain_lab9:main",          # Lab PC
            "controller_lab9 = lab9_pick_place.controller_lab9:main",  # Pi (replaces the course controller)
        ],
    },
)
