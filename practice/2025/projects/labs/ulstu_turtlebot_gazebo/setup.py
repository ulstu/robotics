import os
from setuptools import find_packages, setup

package_name = 'ulstu_turtlebot_gazebo'

def generate_data_files(share_path, dir):
    data_files = []
    for path, _, files in os.walk(dir):
        list_entry = (os.path.dirname(os.path.dirname(share_path)) + '/' + path,
                      [os.path.join(path, f) for f in files if not f.startswith('.')])
        data_files.append(list_entry)

    return data_files

data_files = []
data_files.append(('share/ament_index/resource_index/packages', ['resource/' + package_name]))
data_files.append(('share/' + package_name, ['package.xml']))
data_files += generate_data_files('share/' + package_name + '/launch/', 'launch/')
data_files += generate_data_files('share/' + package_name + '/worlds/', 'worlds/')
data_files += generate_data_files('share/' + package_name + '/resource/', 'resource/')

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools', 'launch'],
    zip_safe=True,
    maintainer='default',
    maintainer_email='default@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ulstu_turtlebot_gazebo_node = ulstu_turtlebot_gazebo.ulstu_turtlebot_gazebo_node:main',
            'scan_pid_node = ulstu_turtlebot_gazebo.scan_pid_node:main'
        ],
    },
)
