from setuptools import setup
import os
from glob import glob

package_name = 'audio_detection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
        (os.path.join("share", package_name), glob("audio_detection/*.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='irene',
    maintainer_email='igonzf06@estudiantes.unileon.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "audio_detector_node = audio_detection.audio_detector_node:main",
            "manager_node = audio_detection.manager_node:main"
        ],
    },
)
