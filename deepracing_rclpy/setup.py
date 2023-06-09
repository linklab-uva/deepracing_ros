from setuptools import find_packages
from setuptools import setup
import os
from glob import glob
package_name = 'deepracing_rclpy'
python_pkg_name = "deepracing_ros"

setup(
    name=package_name,
    version='0.0.0',
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share/' , package_name), ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Trent Weiss',
    author_email='ttw2xk@virginia.edu',
    maintainer='Trent Weiss',
    maintainer_email='ttw2xk@virginia.edu',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'A package for utilizing the F1 Datalogger in Python'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    packages=list(set(find_packages(exclude=['test'])+[
                python_pkg_name
              ])),
    entry_points={
        'console_scripts': [
            'initialize_udp_receiver = %s.scripts.initialize_udp_receiver:main' % (python_pkg_name),
            'initial_raceline_setter = %s.scripts.initial_raceline_setter:main' % (python_pkg_name),
            'admiralnet_path_server = %s.scripts.admiralnet_path_server:main' % (python_pkg_name),
            'oracle_path_server = %s.scripts.oracle_path_server:main' % (python_pkg_name),
            'vehicle_state_publisher = %s.scripts.vehicle_state_publisher:main' % (python_pkg_name),
            'bezier_curve_pure_pursuit = %s.scripts.bezier_curve_pure_pursuit:main' % (python_pkg_name),
            'boundary_publisher = %s.scripts.boundary_publisher:main' % (python_pkg_name),
            'ekf_monitor = %s.scripts.ekf_monitor:main' % (python_pkg_name),
            'xinput_calibration = %s.scripts.xinput_calibration:main' % (python_pkg_name),
            'control_to_xinput = %s.scripts.control_to_xinput:main' % (python_pkg_name),
        ],
    },
)
