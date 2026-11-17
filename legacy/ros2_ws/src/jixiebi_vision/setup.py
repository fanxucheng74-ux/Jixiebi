from setuptools import find_packages, setup

package_name = "jixiebi_vision"

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
    maintainer="Jixiebi maintainers",
    maintainer_email="maintainer@example.invalid",
    description="Read a static image once and publish pixel candidates",
    license="MIT",
    entry_points={"console_scripts": ["static_image = jixiebi_vision.static_image:main"]},
)
