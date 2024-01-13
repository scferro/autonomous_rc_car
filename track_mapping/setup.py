from setuptools import find_packages, setup

package_name = 'track_mapping'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/nav2_params.yaml', 'config/car_nav_urdf.rviz',]),
        ('share/' + package_name + '/launch', ['launch/manual_explore.launch.xml', 'launch/map_track.launch.xml',]),
                                   
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scferro',
    maintainer_email='stephencferro@gmail.com',
    description='A package to drive the autonomous hotlap car to map a track',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['map_track = track_mapping.map_track:main',
        ],
    },
)
