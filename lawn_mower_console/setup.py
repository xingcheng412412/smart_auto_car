from setuptools import find_packages, setup


package_name = "lawn_mower_console"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="lao",
    maintainer_email="lao@todo.todo",
    description="SSH terminal console for mower recording and path tracking.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "lawn_mower_console = lawn_mower_console.main:main",
        ],
    },
)
