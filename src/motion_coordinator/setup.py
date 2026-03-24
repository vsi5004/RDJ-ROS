from setuptools import setup

package_name = "motion_coordinator"

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
    description="Motion coordinator node for the RDJ vinyl robot",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motion_coordinator = motion_coordinator.motion_coordinator_node:main",
        ],
    },
)
