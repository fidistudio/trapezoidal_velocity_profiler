from setuptools import find_packages, setup
import glob

package_name = "velocity_profiler"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["tests*"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ]
    + [("share/" + package_name + "/launch", glob.glob("launch/*.py"))],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="fidel",
    maintainer_email="jorge.martinezram@ingenieria.unam.edu",
    description="Velocity profiler and 2D motion action server",
    license="MIT",
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        "console_scripts": [
            "velocity_profiler_action = velocity_profiler.profiler_action_server:main",
            "goal_pose_bridge = velocity_profiler.goal_pose_bridge:main",
        ],
    },
)
