from setuptools import find_packages, setup

package_name = 'rviz_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/car_urdf.rviz']),
        ('share/' + package_name + '/urdf', ['urdf/car.urdf.xacro', 'urdf/car.gazebo.xacro']),
        ('share/' + package_name + '/env-hooks', ['env-hooks/car.dsv']),
        ('share/' + package_name + '/worlds', ['worlds/car_world.sdf', 'worlds/car_simple.sdf', 'worlds/car_small_oval.sdf']),
        ('share/' + package_name + '/launch', ['launch/car_rviz.launch.py', 'launch/simulate.launch.xml', 'launch/world.launch.xml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='scferro',
    maintainer_email='stephencferro@gmail.com',
    description='A package to show the autonomous hotlap car in RViz and simulate it in Gazebo.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['rviz_description = rviz_description.rviz_description:main'],
    },
)
