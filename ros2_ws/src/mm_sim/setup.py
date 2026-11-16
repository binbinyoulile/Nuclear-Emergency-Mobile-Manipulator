from setuptools import setup

package_name = "mm_sim"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Haobin Zhu",
    maintainer_email="binbinyoulile@users.noreply.github.com",
    description="MuJoCo 仿真桥接节点（物理、传感器、/clock 与帧同步）",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sim_bridge = mm_sim.sim_bridge:main",
        ],
    },
)
