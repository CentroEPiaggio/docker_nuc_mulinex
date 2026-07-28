from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = "socialbot_demo0"


def data_files_under(source_dir: str, install_dir: str):
    entries = []
    root = Path(source_dir)
    if not root.exists():
        return entries

    for path in sorted(root.rglob("*")):
        if path.is_file():
            relative_parent = path.parent.relative_to(root)
            destination = str(Path("share") / package_name / install_dir / relative_parent)
            entries.append((destination, [str(path)]))
    return entries


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ]
    + data_files_under("maps", "maps")
    + data_files_under("worlds", "worlds")
    + data_files_under("assets", "assets"),
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Mulinex Demo 0 Maintainer",
    maintainer_email="maintainer@example.com",
    description="Simulation, navigation, and scripted interview mission for Mulinex OmniCar.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "activate_omni_controller = socialbot_demo0.controller_activator:main",
            "cmd_vel_relay = socialbot_demo0.cmd_vel_relay:main",
            "ground_truth_odom = socialbot_demo0.odom_adapter:main",
            "interview_mission = socialbot_demo0.interview_mission:main",
        ],
    },
)
