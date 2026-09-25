from setuptools import setup

setup(
    name="scout_autoware_interface", version="0.1.0",
    packages=["scout_autoware_interface"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/scout_autoware_interface"]),
        ("share/scout_autoware_interface", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    entry_points={"console_scripts": [
        "scout_interface = scout_autoware_interface.node:main",
    ]},
)
