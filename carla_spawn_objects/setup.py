"""
Setup for carla_spawn_objects
"""
from glob import glob
import os
from pathlib import Path

ROS_VERSION = int(os.environ["ROS_VERSION"])

if ROS_VERSION == 1:
    from distutils.core import setup
    from catkin_pkg.python_setup import generate_distutils_setup

    d = generate_distutils_setup(packages=["carla_spawn_objects"], package_dir={"": "carla_spawn_objects"})

    setup(**d)

elif ROS_VERSION == 2:
    from setuptools import setup

    package_name = "carla_spawn_objects"
    package_dir = Path(".")
    share_root = "share/{}".format(package_name)
    share_config = "{}/config".format(share_root)
    share_launch = "{}/launch".format(share_root)

    setup(
        name=package_name,
        version="0.0.0",
        packages=[package_name],
        data_files=[
            ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
            (share_root, ["package.xml"]),
            (share_config, [str(item) for item in (package_dir / "config").glob("*.json")]),
            (share_launch, [str(item) for item in (package_dir / "launch").glob("*.launch*")]),
        ],
        install_requires=["setuptools"],
        zip_safe=True,
        maintainer="CARLA Simulator Team",
        maintainer_email="carla.simulator@gmail.com",
        description="CARLA spawn_objects for ROS2 bridge",
        license="MIT",
        tests_require=["pytest"],
        entry_points={
            "console_scripts": [
                "carla_spawn_objects = carla_spawn_objects.carla_spawn_objects:main",
                "set_initial_pose = carla_spawn_objects.set_initial_pose:main",
            ]
        },
    )
