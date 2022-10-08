from setuptools import setup
import os
import glob

package_name = 'pnc_ros'

# Create list of data_file tuples by walking through resource folder (exlucding resource index file)
urdf_files = []
for root, directory, file in os.walk(os.getcwd() + "/resource"):
    for f in file:
        if(f != package_name):
            file_rel_path = os.path.join(os.path.relpath(root, os.getcwd()), f)
            install_dir = "share/" + package_name + "/" + os.path.relpath(root, os.getcwd())
            urdf_files.append((install_dir, [file_rel_path]))

data_files = [
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), ["launch/display.launch.py"]),
        (os.path.join('share', package_name, 'launch'), ["launch/pnc.launch.py"]),
        (os.path.join('share', package_name, 'rviz'), ["rviz/pnc_config.rviz"])
    ] + urdf_files

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jwilson',
    maintainer_email='jessemaxxwilson@utexas.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
