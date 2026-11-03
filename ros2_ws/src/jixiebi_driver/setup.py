from glob import glob

from setuptools import find_packages, setup

package_name = "jixiebi_driver"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jixiebi maintainers",
    maintainer_email="maintainer@example.invalid",
    description="Read-only servo status and explicitly reviewed gripper motion service",
    license="MIT",
    entry_points={
        "console_scripts": [
            "raw_state = jixiebi_driver.raw_state:main",
            "nominal_estimate = jixiebi_driver.nominal_estimate:main",
            "single_grip = jixiebi_driver.single_grip:main",
            "single_sweep = jixiebi_driver.single_sweep:main",
            "transfer_preview = jixiebi_driver.transfer_preview:main",
            "transfer_entry_preview = jixiebi_driver.transfer_entry_preview:main",
            "transfer_task = jixiebi_driver.transfer_task:main",
        ]
    },
)
