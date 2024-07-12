from setuptools import setup

package_name = "wrai_eufs_bridge"

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
    description="A bridge between WRAI nodes and EUFS sim.",
    license="All Rights Reserved.",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "cones = wrai_eufs_bridge.cones:main",
            "cone_measures = wrai_eufs_bridge.cone_measures:main",
            "odom = wrai_eufs_bridge.odom:main",
        ],
    },
)
