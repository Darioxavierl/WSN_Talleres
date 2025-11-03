from setuptools import find_packages, setup

package_name = 'dron'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['dron', 'dron.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            "comander_node = dron.comander:main",
            "telemetry_node = dron.telemetry:main",
            "video_node = dron.video:main",
            "mision_node = dron.mission_planer:main",
            "viewer_node=dron.video_viewer:main",
            "failsave_node=dron.battery_failsave:main",
            "video_detector_node=dron.video_detector:main",
            "counter_node=dron.counter:main",
            "checker_node=dron.checker:main"
        ],
    },
)
