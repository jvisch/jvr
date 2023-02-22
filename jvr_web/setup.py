from setuptools import setup
import os

package_name = 'jvr_web'

# copied from https://github.com/ros2/sros2/blob/bb3859098f123c65f8cfc90f45e21502be521d48/sros2/setup.py
def package_files(directory):
    paths = []
    for (path, directories, filenames) in os.walk(directory):
        for filename in filenames:
            paths.append(os.path.join('..', path, filename))
    return paths

extra_files = (
    package_files(f'{package_name}/templates')
)

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    

    package_data={
        package_name : extra_files,
    },

    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='owwwt',
    maintainer_email='owwwt@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'web = jvr_web.web:main'
        ],
    },
)
