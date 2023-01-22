from glob import glob

import os
from setuptools import setup

package_name = 'suave'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gus',
    maintainer_email='g.rezendesilva@tudelft.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pipeline_detection = suave.pipeline_node:main',
            'pipeline_detection_wv = suave_.pipeline_detection_wv:main',
            'spiral_search = suave.spiral_search_lc:main',
            'follow_pipeline = suave.follow_pipeline_lc:main',
            'const_dist_mission_no_adapt = ' +
                'suave.const_dist_mission_no_adapt:main',
            'time_constrained_mission_no_adapt = ' +
                'suave.time_constrained_mission_no_adapt:main',
        ],
    },
)
