from setuptools import setup

package_name = "mm_perception"

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
    description="目标定位节点（深度图 + TF → 粗端中心）",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "target_localizer = mm_perception.target_localizer:main",
        ],
    },
)
