from setuptools import find_packages, setup

package_name = 'pkg_keyboard_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'rclpy', 'std_msgs', 'pynput'],  # 更改为 pynput
    zip_safe=True,
    maintainer='chr',
    maintainer_email='204747508@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_publisher_node = pkg_keyboard_py.keyboard_publisher:main'
        ],
    },
)
