from setuptools import setup

package_name = 'airsim_ros_control'

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
    maintainer='nvidia',
    maintainer_email='juan_lagos_za@mail.toyota.co.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'airsim_ros_control = airsim_ros_control.airsim_ros_control:main',
            'control_test = airsim_ros_control.control_test:main'
        ],
    },
)
