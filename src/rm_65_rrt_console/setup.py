from setuptools import find_packages, setup


package_name = "rm_65_rrt_console"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", ["launch/rrt_console.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="jingling",
    maintainer_email="jingling@example.com",
    description="Console-driven RRT planning, interpolation, and Gazebo execution for the RM65 arm.",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "joint_rrt_console = rm_65_rrt_console.rrt_console_node:main",
        ],
    },
)
