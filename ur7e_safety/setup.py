from setuptools import find_packages, setup

package_name = "ur7e_safety"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cenk",
    maintainer_email="cenkcetix@gmail.com",
    description="Safety helpers for the UR7e stack: wrench-target watchdog.",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "wrench_watchdog = ur7e_safety.wrench_watchdog:main",
        ],
    },
)
