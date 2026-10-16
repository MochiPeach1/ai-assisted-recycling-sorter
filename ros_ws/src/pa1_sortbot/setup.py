from glob import glob
import os

from setuptools import find_packages, setup

package_name = "pa1_sortbot"

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
]
for directory in ("launch", "worlds", "models", "data"):
    for path in glob(os.path.join(directory, "**", "*"), recursive=True):
        if os.path.isfile(path):
            data_files.append((os.path.join("share", package_name, os.path.dirname(path)), [path]))

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Project Maintainer",
    maintainer_email="project@example.invalid",
    description="AI-assisted mobile recycling sorter simulation.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sorter = pa1_sortbot.sorter_node:main",
        ],
    },
)
