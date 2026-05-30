from setuptools import find_packages, setup

package_name = 'hambot_local_nav'

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
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='Local navigation with pluggable controllers',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'sidewalk_segmenter = hambot_local_nav.sidewalk_segmenter:main',
            'voronoi_path_planner = hambot_local_nav.voronoi_path_planner:main',
            'centroid_navigator = hambot_local_nav.centroid_navigator:main',
        ],
    },
)
