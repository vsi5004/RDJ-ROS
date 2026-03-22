from setuptools import setup
package_name = 'diagnostics_aggregator'
setup(
    name=package_name, version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='RDJ Robot', maintainer_email='todo@rdj.local',
    description='Diagnostics aggregator node', license='MIT',
    entry_points={'console_scripts': [
        'diagnostics_aggregator = diagnostics_aggregator.diagnostics_node:main',
    ]},
)
