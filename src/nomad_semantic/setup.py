from glob import glob
from setuptools import find_packages, setup

setup(
    name="nomad_semantic",
    version="0.2.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/nomad_semantic"]),
        ("share/nomad_semantic", ["package.xml", "README.md"]),
        ("share/nomad_semantic/scripts", ["scripts/run.sh"]),
        ("share/nomad_semantic/models/nomad_b0", glob("models/nomad_b0/*")),
    ],
    install_requires=["setuptools"],
    entry_points={"console_scripts": ["semantic = nomad_semantic.node:main"]},
)
