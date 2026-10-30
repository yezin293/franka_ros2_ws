from glob import glob

from setuptools import setup

package_name = 'fr3_skills'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='research_108_01',
    maintainer_email='ecowalker26@gmail.com',
    description='Reusable FR3 motion skills on top of MoveIt',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'move_home = fr3_skills.move_home:main',
            'pose_listener = fr3_skills.pose_listener:main',
            'gripper = fr3_skills.gripper:main',
            'pick_place = fr3_skills.pick_place:main',
            'object_pose_publisher = fr3_skills.object_pose_publisher:main',
            'vision_pick_place = fr3_skills.vision_pick_place:main',
        ],
    },
)
