from setuptools import setup

package_name = 'manipulator_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='you',
    maintainer_email='you@example.com',
    description='Central controller for AMR + manipulator',
    entry_points={
        'console_scripts': [
            'manipulator_node = manipulator_node.manipulator_node:main'
        ],
    },
)
