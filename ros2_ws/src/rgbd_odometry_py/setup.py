from setuptools import find_packages, setup


package_name = "rgbd_odometry_py"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", ["launch/tum_rgbd_py_demo.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="yang07-29",
    maintainer_email="yang07-29@users.noreply.github.com",
    description="Portable ROS 2 Python entry points for RGB-D odometry.",
    license="Proprietary",
    entry_points={
        "console_scripts": [
            "tum_rgbd_publisher = rgbd_odometry_py.entrypoints:tum_publisher",
            "rgbd_odometry_py_node = rgbd_odometry_py.entrypoints:odometry",
        ],
    },
)
