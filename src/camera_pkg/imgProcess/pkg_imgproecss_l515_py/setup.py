from setuptools import find_packages, setup

package_name = 'pkg_imgproecss_l515_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chr',
    maintainer_email='204747508@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_detect_node = pkg_imgproecss_l515_py.image_detect_node:main',
            # --- 新增的调试脚本入口点 ---
            'debug_env = pkg_imgproecss_l515_py.debug_env:main'
            # --- 确保 `debug_env.py` 文件中有一个 `main()` 函数，否则可以省略 `:main` ---
        ],
    },
)
