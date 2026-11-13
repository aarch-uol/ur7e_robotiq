from glob import glob

from setuptools import find_packages, setup

package_name = "moveit_gui"

setup(
    name=package_name,
    version="2.0.0",
    packages=find_packages(exclude=["test", "legacy"]),
    # the web front end ships inside the Python package (three.js for the 3D view under static/vendor)
    package_data={"moveit_gui.web": ["static/*", "static/vendor/three/*", "static/vendor/three/build/*",
                                     "static/vendor/three/examples/jsm/loaders/*",
                                     "static/vendor/three/examples/jsm/controls/*"]},
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/profiles", glob("profiles/*.yaml")),
        ("share/" + package_name + "/programs", glob("programs/*.yaml")),
        ("share/" + package_name + "/policies", glob("policies/*")),
    ],
    install_requires=["setuptools", "PyYAML", "numpy"],
    zip_safe=True,
    maintainer="Dzhenk Emurla",
    maintainer_email="cenk.cetin@liverpool.ac.uk",
    description="Qt operator panel for MoveIt 2 + ur_robot_driver (ROS 2 Jazzy)",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["moveit_gui = moveit_gui.app:main",
                                      "moveit_gui_web = moveit_gui.web.server:run",
                                      "moveit_gui_idle = moveit_gui.web.idle:run"]},
)
