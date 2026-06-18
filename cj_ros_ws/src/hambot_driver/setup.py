from setuptools import setup

package_name = 'hambot_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/hambot_driver.launch.py']),
        ('share/' + package_name + '/config', ['config/hambot_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Chance Hamilton',
    maintainer_email='chamilton@usf.edu',
    description='ROS 2 hardware drivers for HamBot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_driver = hambot_driver.motor_driver:main',
            'motor_test = hambot_driver.motor_test:main',
        ],
    },
)