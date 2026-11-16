from setuptools import setup

package_name = "mm_mission"

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
    description="双车协同任务决策层（状态机与控制律，硬件无关）",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mission_node = mm_mission.node:main",
        ],
    },
)
