from glob import glob

from setuptools import setup

package_name = 'roboracer_dashboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # The web page is installed as data files. With `colcon build --symlink-install`
        # they are symlinks to src/, so editing HTML/JS only needs a browser refresh.
        # (Adding a NEW file still needs a rebuild.)
        ('share/' + package_name + '/web', glob('web/*.html') + glob('web/*.css')),
        ('share/' + package_name + '/web/js', glob('web/js/*.js')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sebas',
    maintainer_email='sebastianmedina063@gmail.com',
    description='Web dashboard for the RoboRacer (camera, LiDAR, odometry, SLAM map).',
    license='MIT',
    entry_points={
        'console_scripts': [
            'web_server = roboracer_dashboard.web_server:main',
            'video_server = roboracer_dashboard.video_server:main',
        ],
    },
)
