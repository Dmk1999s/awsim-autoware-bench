from setuptools import setup

package_name = "autoware_bench"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="loik1235",
    maintainer_email="loik1235@gmail.com",
    description="AWSIM + Autoware 주행을 1회 단위로 기록하는 계측 도구",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "metrics_collector = autoware_bench.metrics_collector:main",
        ],
    },
)
