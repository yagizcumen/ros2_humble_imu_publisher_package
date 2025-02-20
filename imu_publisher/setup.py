from setuptools import setup

package_name = 'imu_publisher'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yağız',
    maintainer_email='ornek@ktu.edu.tr',
    description='IMU verilerini ROS2 topic olarak yayınlayan node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_publisher = imu_publisher.imu_publisher:main',
        ],
    },
)
