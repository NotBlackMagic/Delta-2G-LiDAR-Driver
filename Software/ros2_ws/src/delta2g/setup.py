from setuptools import setup

package_name = 'delta2g'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='NotBlackMagic',
    maintainer_email='social@notblackmagic.com',
    description='Simple LidarScan publisher for Delta-2G Lidar',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'talker = delta2g.LiDARConnectionROS:main',
        ],
    },
)
