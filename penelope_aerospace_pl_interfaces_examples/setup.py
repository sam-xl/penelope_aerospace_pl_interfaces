from setuptools import setup

package_name = "penelope_aerospace_pl_interfaces_examples"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        # ("lib/" + package_name, [package_name + "/example_action_client.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Eugenio Bernardi",
    maintainer_email="e.bernardi@tudelft.nl",
    description="Simple example of a ROS2 action server and client for the PeneloPe specific interfaces",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "example_client = penelope_aerospace_pl_interfaces_examples.example_action_client:main",
            "example_server = penelope_aerospace_pl_interfaces_examples.example_action_server:main",
            "induct_serverr = penelope_aerospace_pl_interfaces_examples.induct_action_server:main",
        ],
    },
)
