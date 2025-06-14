from setuptools import setup
import os
from glob import glob

package_name = 'agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('lib', package_name), glob(f'{package_name}/*.py')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michal Peterka',
    maintainer_email='peterka.m@icloud.com',
    description='Agent for f1tenth_gym_ros',
    license='MIT',
    entry_points={
        'console_scripts': [
            'purepursuitagent = agent.purepursuitagent:main',
            'samplingagent = agent.samplingagent:main',
            'samplingagentlimited = agent.samplingagentlimited:main',
            'reactiveagent = agent.reactiveagent:main',
            'detector = agent.detector:main',
        ],
    },
)
