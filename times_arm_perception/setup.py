from glob import glob

from setuptools import find_packages, setup

setup(
    name="times_arm_perception",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/times_arm_perception"],
        ),
        ("share/times_arm_perception", ["package.xml"]),
        ("share/times_arm_perception/config", glob("config/*.yaml")),
        ("share/times_arm_perception/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    maintainer="Rainy-Day04",
    maintainer_email="rainy-day04@users.noreply.github.com",
    description="Perception and manual grasp coordination over the existing arm API.",
    license="Apache-2.0",
    entry_points={"console_scripts": ["coordinator = times_arm_perception.node:main"]},
)
