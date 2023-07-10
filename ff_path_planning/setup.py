from setuptools import setup, find_packages

package_name = "ff_path_planning"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rdyro",
    maintainer_email="rdyro@stanford.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "path_planning = ff_path_planning.path_planning:main",
            "examples:service_call = ff_path_planning.examples.service_call:main",
            "examples:setting_goal = ff_path_planning.examples.setting_goal:main",
            "examples:path_planning = ff_path_planning.examples.path_planning:main",
        ],
    },
)
