import os
from glob import glob

from setuptools import setup

package_name = 'easy_handeye2'

setup(
    name=package_name,
    version='0.5.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name + '/resource', glob(os.path.join('resource', '*.ui'))),
        ('share/' + package_name, ['package.xml']),
        # 确保这些 .xml 文件也被正确安装，它们通常是 rqt 插件的定义文件
        ('share/' + package_name, ['plugin_evaluator.xml', 'plugin_calibrator.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marco Esposito',
    maintainer_email='esposito@imfusion.com',
    description='Simple, hardware-independent ROS2 library for hand-eye calibration',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'handeye_server = easy_handeye2.handeye_server:main',
            'handeye_server_robot = easy_handeye2.handeye_server_robot:main',
            'handeye_publisher = easy_handeye2.handeye_publisher:main',
            'handeye_calibration_commander = easy_handeye2.handeye_calibration_commander:main',
        ],
        'rqt_gui_py_plugins': [
            'RqtHandeyeCalibrator = easy_handeye2.handeye_rqt_calibrator:RqtHandeyeCalibrator',
            'RqtHandeyeEvaluator = easy_handeye2.handeye_rqt_evaluator:RqtHandeyeEvaluator',
        ],
    },
    # # 关键修改：修正 scripts 列表中的路径，指向 `scripts/` 子目录下的实际脚本文件
    # scripts=[
    #     'scripts/rqt_calibrator.py', # 修正路径：现在直接是 'scripts/' 目录下的文件
    #     'scripts/rqt_evaluator.py',  # 修正路径
    # ]
)
