import os

from setuptools import find_packages, setup
from setuptools.glob import glob

package_name = 'autorace'


def package_files(directory):
    data_files = []
    for path, _, filenames in os.walk(directory):
        if not filenames:
            continue
        install_path = os.path.join('share', package_name, path)
        files = [os.path.join(path, filename) for filename in filenames]
        data_files.append((install_path, files))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.xml')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
    ] + package_files('models'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alex',
    maintainer_email='aleksei.kazinskii@rokolabs.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'autorace_node = autorace.autorace_node:main'
        ],
    },
)
