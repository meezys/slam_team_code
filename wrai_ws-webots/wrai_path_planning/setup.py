from setuptools import setup

package_name = 'wrai_path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chenson Hope-Simes',
    maintainer_email='Chenson.Hope-Simes@warwick.ac.uk',
    description='A ROS package that generates the path through a set of cones.',
    license='All Rights Reserved.',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'midpoints = wrai_path_planning.midpoints:main'
        ],
    },
)
