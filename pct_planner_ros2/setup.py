from setuptools import setup

package_name = "pct_planner_ros2"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            ["launch/planner.launch.py"],
        ),
        (
            "share/" + package_name + "/config",
            ["config/planner.yaml"],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "planner_node = pct_planner_ros2.planner_node:main",
        ],
    },
)
