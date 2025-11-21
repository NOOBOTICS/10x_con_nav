from setuptools import find_packages, setup

package_name = 'tb_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # ('share/' + package_name + '/maps', [
        #     'maps/nav_map.yaml',
        #     'maps/nav_map.pgm',
        # ])

        # ('share/' + package_name + '/rviz', ['rviz/rviz_config.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nobel',
    maintainer_email='22ee01062@iitbbs.ac.in',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'trajectory_publisher = tb_controller.trajectory_publisher:main',
            'trajectory_controller = tb_controller.trajectory_controller:main',

        ],
    },
)
