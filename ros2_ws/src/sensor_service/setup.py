from setuptools import find_packages, setup

package_name = 'sensor_service'

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
    maintainer='kam',
    maintainer_email='cagomez@usc.edu',
    description='Service server to continuously read data from the sensor',
    license='BSD-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor = sensor_service.sensor:main',
        ],
    },
)
