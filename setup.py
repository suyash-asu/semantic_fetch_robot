from setuptools import find_packages, setup

package_name = "semantic_fetch_robot"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages",
            ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Group 2",
    maintainer_email="sdhir5@asu.edu",
    description="""
    ROS 2 package for a semantic fetch robot capable of receiving high-level
    object requests and navigating in a simulated environment to retrieve them.
    This package will include nodes for request handling, semantic mapping,
    and robot navigation.
    """,
    license="Apache-2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
    "console_scripts": [
        "semantic_fetch_node = semantic_fetch_robot.semantic_fetch_node:main",
        "noise_injector_node = semantic_fetch_robot.noise_injector_node:main",
        "navigate_to_goal = semantic_fetch_robot.navigate_to_goal:main",
        ],
    },
)
