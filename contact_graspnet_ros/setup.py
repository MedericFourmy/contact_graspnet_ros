from pathlib import Path
from typing import List

from setuptools import find_packages, setup

from generate_parameter_library_py.setup_helper import generate_parameter_module

package_name = "contact_graspnet_ros"
project_source_dir = Path(__file__).parent

module_name = "contact_graspnet_ros_parameters"
yaml_file = "contact_graspnet_ros/contact_graspnet_ros_parameters.yaml"
generate_parameter_module(module_name, yaml_file)


def get_files(dir: Path, pattern: str) -> List[str]:
    return [x.as_posix() for x in (dir).glob(pattern) if x.is_file()]


setup(
    name=package_name,
    version="0.0.2",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mederic Fourmy",
    maintainer_email="mederic.fourmy@gmail.com",
    description="ROS 2 wrapper around contact_graspnet python library for 6D grasp estimation",
    license="BSD",
    entry_points={
        "console_scripts": [
            "contact_graspnet_node = contact_graspnet_ros.contact_graspnet_node:main",
        ],
    },
)
