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
        (os.path.join('share/' , package_name, "launch"), glob(os.path.join("launch", "*.launch*"))),
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
                # os.path.join(python_pkg_name,"controls"),
                # os.path.join(python_pkg_name,"convert"),
              ])),
   # data_files=[
       # ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        # Include our package.xml file
       # (os.path.join('share', package_name), ['package.xml']),
        # Include all launch files.
      #  (os.path.join('share', package_name, 'launch'), glob('*.launch.py'))
    #],
    entry_points={
        'console_scripts': [
            'pose_publisher = %s.scripts.pose_publisher:main' % (python_pkg_name),
            'bezier_rviz = %s.scripts.bezier_rviz:main' % (python_pkg_name),
            'waypoint_plot_recorder = %s.scripts.record_plots_waypoint:main' % (python_pkg_name),
            'pure_pursuit_bezier = %s.scripts.admiralnet_bezier_script:main' % (python_pkg_name),
            'pure_pursuit_waypoint = %s.scripts.admiralnet_waypoint_script:main' % (python_pkg_name),
            'pure_pursuit_oracle = %s.scripts.oracle_pure_pursuit_script:main' % (python_pkg_name),
            'pure_pursuit_trajopt = %s.scripts.trajopt_script:main' % (python_pkg_name),
            'pilotnet = %s.scripts.pilotnet_script:main' % (python_pkg_name),
            'cnnlstm = %s.scripts.cnnlstm_script:main' % (python_pkg_name),
            'admiralnet_e2e = %s.scripts.admiralnet_e2e_script:main' % (python_pkg_name),
            'generate_steering_calibration = %s.scripts.generate_steering_calibration:main' % (python_pkg_name),
            'point_cloud_display = %s.scripts.point_cloud_display:main' % (python_pkg_name),
            'velocity_publisher = %s.scripts.velocity_publisher:main' % (python_pkg_name),
            'vjoy_control_node = %s.scripts.vjoy_control_node:main' % (python_pkg_name),
            'tf2_test = %s.scripts.tf2_test:main' % (python_pkg_name),
            
            
        ],
    },
)
