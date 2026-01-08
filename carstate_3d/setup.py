from setuptools import setup
import os
from glob import glob

package_name = 'carstate_3d'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launch 파일 설치 추가
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jeonseoknam',
    maintainer_email='j20nsuknam@gmail.com',
    description='Carstate node replacement that uses EKF outputs and adds Frenet conversion.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carstate_3d_node = carstate_3d.carstate_3d_node:main'
        ],
    },
)

