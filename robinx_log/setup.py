from setuptools import setup

package_name = 'robinx_log'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/rtkgpslog_web_viewer.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shaohua He',
    maintainer_email='shaohuahe@robinx.com',
    description='RTKGPSLog 实时数据查看服务器 (ROS 2)',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rtkgpslog_web_viewer = robinx_log.serve_rtkgpslog_node:main',
        ],
    },
)
