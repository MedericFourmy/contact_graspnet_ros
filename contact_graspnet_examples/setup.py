from pathlib import Path
from typing import List
from setuptools import find_packages, setup

package_name = "contact_graspnet_examples"

project_source_dir = Path(__file__).parent

def get_files(dir: Path, pattern: str) -> List[str]:
    return [x.as_posix() for x in (dir).glob(pattern) if x.is_file()]


setup(
    name=package_name,
    version="0.0.2",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            f"share/{package_name}/launch",
            get_files(project_source_dir / "launch", "*.launch.py"),
        ),
        (
            f"share/{package_name}/config",
            get_files(project_source_dir / "config", "*.yaml"),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mederic Fourmy",
    maintainer_email="mederic.fourmy@gmail.com",
    description="ROS 2 example node for contact_graspnet_ros",
    license="BSD",
    entry_points={
        "console_scripts": [
            "client = contact_graspnet_examples.client:main",
        ],
    },
)
