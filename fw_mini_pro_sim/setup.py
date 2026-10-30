from pathlib import Path
from setuptools import setup

package_name = 'fw_mini_pro_sim'
data_files = [
    ('share/ament_index/resource_index/packages',['resource/'+package_name]),
    ('share/'+package_name,['package.xml','README.md','VALIDATION.md','SLAM.md']),
]
for directory in ('launch','models','worlds','config','paths','scripts','validation'):
    for path in sorted(Path(directory).rglob('*')):
        if path.is_file() and '__pycache__' not in path.parts:
            data_files.append(('share/'+package_name+'/'+str(path.parent),[str(path)]))
setup(name=package_name,version='0.1.0',packages=[package_name],
      data_files=data_files,install_requires=['setuptools'],zip_safe=True,
      maintainer='FW-mini-pro workspace maintainers',maintainer_email='maintainer@example.com',
      description='Four-wheel steering Gazebo Fortress simulation',license='Proprietary',
      tests_require=['pytest'],entry_points={'console_scripts':[
          'slam_rtk_fusion = fw_mini_pro_sim.fusion_node:main',
          'rtk_simulator = fw_mini_pro_sim.rtk_simulator:main',
          'simulation_adapter = fw_mini_pro_sim.adapter:main']})
