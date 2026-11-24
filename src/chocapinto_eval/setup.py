from glob import glob

from setuptools import setup

package_name = 'chocapinto_eval'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sebas',
    maintainer_email='sebastianmedina063@gmail.com',
    description='Automatic evaluation of simulation runs: laps, cone contacts, odometry drift.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'race_monitor = chocapinto_eval.race_monitor_node:main',
        ],
    },
)
