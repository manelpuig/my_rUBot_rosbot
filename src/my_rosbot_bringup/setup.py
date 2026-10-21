from glob import glob
import os

from setuptools import setup

package_name = "my_rosbot_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Manel Puig",
    maintainer_email="manel.puig@ub.edu",
    description="Project launch wrappers for the official ROSbot XL stack.",
    license="MIT",
    entry_points={"console_scripts": []},
)
