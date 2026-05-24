from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'modeloo_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'data'), glob('data/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='gabriel',
    maintainer_email='gabriel@todo.todo',
    description='SCARA robot description package with URDF, meshes, and launch files',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'ejecutar_trayectoria = modeloo_robot.ejecutar_trayectoria:main',
    ],
},
)