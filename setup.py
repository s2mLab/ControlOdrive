"""
This file is used to build the project.
"""
from setuptools import setup

setup(
    name='ergocycleS2M',
    version='1.0',
    author='S2M',
    description='A project to control the ergocycle with a graphic interface',
    packages=[
        'ergocycleS2M',
        'ergocycleS2M.data_processing',
        'ergocycleS2M.motor_control',
        'ergocycleS2M.gui',
        'ergocycleS2M.parameters'
    ],
    data_files=[
            ('ergocycleS2M/parameters',
             ['ergocycleS2M/parameters/hardware_and_security.json', 'ergocycleS2M/parameters/gains.json']),
    ],
    install_requires=['numpy', 'matplotlib'],
    entry_points={
        'console_scripts': [
            'read_ergocycle_file = ergocycleS2M.data_processing.save_and_load:read_from_terminal',
            'start_ergocycle = ergocycleS2M.gui.main:main',
        ],
    },
)
