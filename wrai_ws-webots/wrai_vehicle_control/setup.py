from setuptools import setup

package_name = "wrai_vehicle_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Joshua Jose",
    maintainer_email="Joshua.Jose.1@warwick.ac.uk",
    description="Generates control commands for a car, given a path",
    license="All Rights Reserved.",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["pure_pursuit = wrai_vehicle_control.main:main"],
    },
)
