from glob import glob
from setuptools import setup

setup(
    name="scout_ht_traversability", version="0.1.0",
    packages=["scout_ht_traversability"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/scout_ht_traversability"]),
        ("share/scout_ht_traversability", ["package.xml"]),
        ("share/scout_ht_traversability/config", glob("config/*.yaml")),
        ("share/scout_ht_traversability/launch", glob("launch/*.py")),
        ("share/scout_ht_traversability/rviz", glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools", "numpy"],
    entry_points={"console_scripts": [
        "costmap_ht_fusion = scout_ht_traversability.node:main",
        "global_elevation_mapper = scout_ht_traversability.elevation_manager:main",
    ]},
)
