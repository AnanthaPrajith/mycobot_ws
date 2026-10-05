from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mycobot_brain'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools',
                      "scipy",
                      "numpy",
    ],
    zip_safe=True,
    maintainer='yannick',
    maintainer_email='yannick@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'brain = mycobot_brain.brain:main',
            'brain_test = mycobot_brain.brain_test:main',
            'jog_robot = mycobot_brain.jog_robot:main',
        ],
    },
)
