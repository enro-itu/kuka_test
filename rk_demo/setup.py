from setuptools import setup
import os
from glob import glob

package_name = 'rk_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # Launch dosyaları
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),

        # URDF dosyaları
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),

        # Mesh dosyaları (hem STL hem stl)
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.STL')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yagizcumen',
    maintainer_email='yagizcumen@example.com',
    description='Demo robot arm package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)
