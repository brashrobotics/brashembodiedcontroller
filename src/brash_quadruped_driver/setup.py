from glob import glob
from setuptools import find_packages, setup


package_name = "brash_quadruped_driver"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Brash Robotics",
    maintainer_email="engineering@brashrobotics.com",
    description="Brash adapters and safety gates for the Unitree B2W platform.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "unitree_imu_bridge = brash_quadruped_driver.unitree_imu_bridge:main",
            "velocity_guard = brash_quadruped_driver.velocity_guard:main",
            "b2w_teleop_sim_bridge = brash_quadruped_driver.b2w_teleop_sim_bridge:main",
        ],
    },
)
