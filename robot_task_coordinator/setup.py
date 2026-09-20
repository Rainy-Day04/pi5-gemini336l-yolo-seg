from glob import glob

from setuptools import find_packages, setup

setup(
    name="robot_task_coordinator",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/robot_task_coordinator"],
        ),
        ("share/robot_task_coordinator", ["package.xml"]),
        ("share/robot_task_coordinator/config", glob("config/*.yaml")),
        ("share/robot_task_coordinator/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    maintainer="Rainy-Day04",
    maintainer_email="rainy-day04@users.noreply.github.com",
    description="Visual selection, navigation and arm handoff through ROS actions.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "coordinator = robot_task_coordinator.node:main",
            "select_target = robot_task_coordinator.select:main",
        ]
    },
)
