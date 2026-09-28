from setuptools import find_packages, setup

package_name = 'mycobot_controller'

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
                      "pymycobot",
                      "RPi.GPIO",
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
            'controller = mycobot_controller.controller:main',
            'test = mycobot_controller.test:main',
        ],
    },
)
