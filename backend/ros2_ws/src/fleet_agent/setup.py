import os
from glob import glob

from setuptools import setup

package_name = "fleet_agent"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="FLEET-X team",
    maintainer_email="fleetx@example.com",
    description="One FLEET-X robot as a ROS 2 node, driven by the shared brain.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "agent = fleet_agent.agent_node:main",
            "order_source = fleet_agent.order_source:main",
        ],
    },
)
