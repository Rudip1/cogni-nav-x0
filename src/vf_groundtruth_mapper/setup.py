from setuptools import setup
from glob import glob
import os

package_name = "gazebo_2d_groundtruth_map"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/resource", glob("resource/*")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/worlds", glob("worlds/*.world")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Pravin Oli",
    maintainer_email="olipravin18@gmail.com",
    description="Gazebo 2D ground-truth map plugin with C++ and Python nodes",
    license="Apache-2.0",
    extras_require={
        "test": ["pytest"],
    },
)
