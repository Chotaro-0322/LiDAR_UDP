from setuptools import setup

package_name = 'lidar_udp_catch'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='itolab-chotaro',
    maintainer_email='bq17088@shibaura-it.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ "lidar_udp_catch = lidar_udp_catch.main:main"
        ],
    },
)
