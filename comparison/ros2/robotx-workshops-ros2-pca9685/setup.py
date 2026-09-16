from setuptools import find_packages, setup

package_name = "pca9685"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "adafruit-pca9685"],
    zip_safe=True,
    maintainer="Andrew Johnson",
    maintainer_email="andrewmjohnson549@gmail.com",
    description="A package for sening pulse width modulation signals to a PCA9685 board",
    license="Apache License 2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["pca9685_node = pca9685.pca9685_node:main"],
    },
)
