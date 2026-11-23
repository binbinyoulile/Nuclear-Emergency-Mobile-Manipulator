from setuptools import setup

package_name = "mm_base"

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
    description="真机底盘 CAN 驱动（SocketCAN、四轮差速里程计、无硬件自检）",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "base_node = mm_base.base_node:main",
            "selftest = mm_base.selftest:main",
        ],
    },
)
