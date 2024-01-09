import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'assignment_template'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, "launch"), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, "worlds"), glob('worlds/*.world')),
        (os.path.join('share', package_name, "params"), glob('params/*.yaml')),
        (os.path.join('share', package_name, "maps"), glob('maps/*.pgm') + glob('maps/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gcielniak',
    maintainer_email='gcielniak@lincoln.ac.uk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'example_nav_to_pose = assignment_template.example_nav_to_pose:main',
            'object_detector = assignment_template.object_detector:main',
            'image_colour_filter = assignment_template.image_colour_filter:main',
            'image_projection_3 = assignment_template.image_projection_3:main',
            'pothole_clustering = assignment_template.pothole_clustering:main',
            'yolo_pothole = assignment_template.yolo_pothole:main',
            'save_video = assignment_template.save_video:main',
            'movement = assignment_template.movement:main',
            'movement_2 = assignment_template.movement_2:main',
            'follow_path = assignment_template.follow_path:main',
        ],
    },
)
