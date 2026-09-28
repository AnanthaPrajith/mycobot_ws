from setuptools import setup

package_name = 'mycobot_control'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mycobot_control.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@todo.todo',
    description='Unified control node for myCobot (velocity/position/ee pose).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'mycobot_control = mycobot_control.mycobot_control:main',
        ],
    },
)
