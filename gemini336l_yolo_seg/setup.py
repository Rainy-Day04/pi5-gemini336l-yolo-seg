from setuptools import find_packages, setup

package_name = 'gemini336l_yolo_seg'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rainy-Day04',
    maintainer_email='rainy-day04@users.noreply.github.com',
    description='Non-blocking YOLO segmentation and RGB-D 3D projection.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['seg_node = gemini336l_yolo_seg.node:main']},
)
