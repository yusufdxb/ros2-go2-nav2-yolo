from setuptools import setup

package_name = 'go2_yolo_detector'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('lib/' + package_name, ['scripts/detector_node']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'detector_node = go2_yolo_detector.detector_node:main',
            'sim_person_detector = go2_yolo_detector.sim_person_detector:main',
        ],
    },
)
