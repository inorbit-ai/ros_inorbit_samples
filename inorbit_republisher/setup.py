from setuptools import setup

package_name = 'inorbit_republisher'

setup(
    name=package_name,
    version='0.2.5',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='InOrbit',
    maintainer_email='support@inorbit.ai',
    description='ROS2 to InOrbit topic republisher',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'republisher = inorbit_republisher.republisher:main'
        ],
    },
)
