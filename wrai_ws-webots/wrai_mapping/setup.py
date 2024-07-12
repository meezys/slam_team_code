import os
from glob import glob
from setuptools import setup

package_name = "wrai_mapping"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Joshua Jose",
    maintainer_email="Joshua.Jose.1@warwick.ac.uk",
    description="Maps cones and determines vehicle position",
    license="All Rights Reserved",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cone_slam = wrai_mapping.main:main",
            "chassis_tf = wrai_mapping.chassis_tf:main",
        ],
    },
)
