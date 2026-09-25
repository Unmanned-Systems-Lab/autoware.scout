from setuptools import find_packages
from setuptools import setup


package_name = "ins_driver_ez21"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", ["config/driver.yaml"]),
        (
            f"share/{package_name}/launch",
            ["launch/ins_driver.launch.py", "launch/ins_localization.launch.py"],
        ),
        (f"share/{package_name}", ["README.md"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="nvidia",
    maintainer_email="nvidia@example.com",
    description="ROS 2 INS driver for reading CAN frames and publishing odometry.",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ins_driver_node = ins_driver_ez21.ins_driver_node:main",
            "ins_localization_bridge = ins_driver_ez21.ins_localization_bridge:main",
            "planning_fallback_publishers = ins_driver_ez21.planning_fallback_publishers:main",
            "freespace_bypass_coordinator = ins_driver_ez21.freespace_bypass_coordinator:main",
            "extract_gps_from_can_txt = ins_driver_ez21.extract_gps_from_can_txt:main",
        ],
    },
)
