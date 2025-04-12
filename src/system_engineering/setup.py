from setuptools import find_packages, setup

package_name = 'system_engineering'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='teja',
    maintainer_email='teja2022ai@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'system_monitor = system_engineering.system_monitor:main',
            'command_publisher = system_engineering.command_publisher:main',
            'command_processor = system_engineering.command_processor:main'
        ],
    },
)
