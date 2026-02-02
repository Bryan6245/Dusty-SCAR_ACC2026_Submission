from setuptools import find_packages, setup

package_name = 'dusty_scar'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bryan',
    maintainer_email='bryanherrera799@gmail.com',
    description='Dusty SCAR ACC2026 autonomy nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_nav = dusty_scar.lidar_nav:main',
            'lane_follow = dusty_scar.lane_follow:main',
            'exit_then_lane = dusty_scar.exit_then_lane:main',
        ],
    },
)