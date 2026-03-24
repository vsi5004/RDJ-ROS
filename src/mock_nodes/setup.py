from setuptools import setup

package_name = "mock_nodes"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="RDJ Robot",
    maintainer_email="todo@rdj.local",
    description="Mock CAN nodes for simulation",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mock_canopen_master = mock_nodes.mock_canopen_master:main",
        ],
    },
)
