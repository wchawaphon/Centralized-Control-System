from setuptools import setup

package_name = 'amr_node'

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
            'amr_node = amr_node.amr_node:main'
        ],
    },
)
