from setuptools import setup, find_packages

setup(
    name='ros2tools',
    version='0.1.0',
    description="A command line tool for inspecting ROS2 nodes and topics and generating json output",
    long_description="A command line tool for inspecting ROS2 nodes and topics and generating json output",
    packages=find_packages(include=['ros2tools', 'ros2tools.*']),
    include_package_data=True,
    install_requires=[
    ],
    entry_points={
        'console_scripts': [
            'ros2-node-inspector = ros2tools.ros2_node_inspector:main',
            'ros2-node-grapher = ros2tools.ros2_node_grapher:main',
            'ros2-tracer = ros2tools.ros2_tracer:main',
        ],
    },
    package_data={
    },
    zip_safe=False,
)

