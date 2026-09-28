from setuptools import find_packages, setup

package_name = 'mycobot_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',
                      "numpy",
                      "opencv-python",
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
            'vision = mycobot_vision.vision:main',
            'vision2 = mycobot_vision.vision2:main',
        ],
    },
)
