from setuptools import setup

package_name = "wrai_perception"

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
    maintainer="Raunaq Singh",
    maintainer_email="Raunaq.Singh@warwick.ac.uk",
    description="A ROS package that detects cones in the cars environment.",
    license="All Rights Reserved.",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["yolo = wrai_perception.yolo:main"],
    },
)
