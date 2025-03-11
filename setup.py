from setuptools import find_packages, setup

package_name = 'turtlebot3_baller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/ball_and_goal.launch.py']),
        ('share/' + package_name + '/launch', ['launch/spawn_turtlebot3.launch.py']),
        ('share/' + package_name + '/launch', ['launch/robot_state_publisher.launch.py']),
        ('share/' + package_name + '/worlds', ['worlds/football.world']),
        ('share/' + package_name + '/urdf', ['urdf/turtlebot3_baller.urdf']),
        ('share/' + package_name + '/models/ball', ['models/ball/model.sdf']),
        ('share/' + package_name + '/models/goal', ['models/goal/model.sdf']),
        ('share/' + package_name + '/models/turtlebot3_baller', ['models/turtlebot3_baller/model.sdf']),
        ('share/' + package_name + '/urdf', ['urdf/turtlebot3_baller.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='greenhub',
    maintainer_email='greenhub@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ball_and_goal = turtlebot3_baller.ball_and_goal:main',
        ],
    },
)
