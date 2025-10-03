import os
from setuptools import find_packages, setup
from glob import glob

package_name = 'opera_modules'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'params'),glob('params/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kasahara',
    maintainer_email='kasahara.yuichiro.res@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'watchdog_ekf_input = opera_modules.watchdog_ekf_input:main',
            'soil_release_module = opera_modules.soil_release_module:main',
            'topic_relay_gui = opera_modules.topic_relay_gui:main'
        ],
    },
)
