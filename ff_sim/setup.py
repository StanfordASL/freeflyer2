from glob import glob
from setuptools import setup

package_name = "ff_sim"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*launch.[pxy][yma]*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="alvin",
    maintainer_email="alvinsunyixiao@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
                            "simulator_node = ff_sim.simulator_node:main",
                            "controller_metrics = ff_sim.controller_metrics:main"
                            ],
    },
)
