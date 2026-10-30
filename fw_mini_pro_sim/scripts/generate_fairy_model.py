#!/usr/bin/env python3
"""Generate an independent Fairy approximation and a mapping arena."""
from pathlib import Path
import math
import xml.etree.ElementTree as E
from generate_model import child, inertia, geometry as base_geometry, plugin
import yaml

ROOT = Path(__file__).resolve().parents[1]
MOUNT_Z = .483976069

def geometry(*args, **kwargs):
    base_geometry(*args, **kwargs)
    link = args[0]
    for visual in link.findall('visual'):
        visual.set('name', visual.get('name') + '_visual')

def main():
    tree = E.parse(ROOT/'models/fw_mini_pro/model.sdf')
    model = tree.getroot().find('model')
    model.set('name','fw_mini_pro_fairy')
    support = child(model,'link',name='lidar_support')
    child(support,'pose',f'0 0 {MOUNT_Z-.15} 0 0 0')
    inertia(support,.15,(.0012,.0012,.00003))
    geometry(support,'post','cylinder',{'radius':.012,'length':.3},'.3 .3 .3 1')
    lidar = child(model,'link',name='rslidar')
    child(lidar,'pose',f'0 0 {MOUNT_Z} 0 0 0')
    inertia(lidar,.35,(.000266,.000266,.000246))
    child(lidar.find('inertial'),'pose','0 0 .035 0 0 0')
    geometry(lidar,'fairy_body','cylinder',{'radius':.0375,'length':.07},'.15 .2 .25 1','0 0 .035 0 0 0')
    for name,kid in [('support_mount_joint','lidar_support'),('lidar_mount_joint','rslidar')]:
        j=child(model,'joint',name=name,type='fixed')
        child(j,'parent','base_link');child(j,'child',kid)
    s=child(lidar,'sensor',name='fairy',type='gpu_lidar')
    # Optical centre approximation; rslidar remains the documented base origin.
    child(s,'pose','0 0 .035 0 0 0')
    child(s,'topic','/fairy');child(s,'gz_frame_id','fairy_optical')
    child(s,'always_on','true');child(s,'update_rate',10);child(s,'visualize','false')
    ray=child(s,'lidar');scan=child(ray,'scan')
    for axis,n,lo,hi in [('horizontal',1440,-math.pi,math.pi),('vertical',96,math.radians(-16),math.radians(16))]:
        a=child(scan,axis)
        for k,v in [('samples',n),('resolution',1),('min_angle',lo),('max_angle',hi)]:child(a,k,v)
    r=child(ray,'range')
    for k,v in [('min',.1),('max',150),('resolution',.01)]:child(r,k,v)
    dest=ROOT/'models/fw_mini_pro_fairy';dest.mkdir(exist_ok=True)
    E.indent(tree);tree.write(dest/'model.sdf',encoding='unicode',xml_declaration=True)
    (dest/'model.config').write_text('<model><name>fw_mini_pro_fairy</name><version>1.0</version><sdf version="1.8">model.sdf</sdf><description>Fairy approximation on 0.3 m roof support</description></model>')
    worldtree=E.parse(ROOT/'worlds/test_ground.sdf');w=worldtree.getroot().find('world')
    w.set('name','slam_ground');w.find('include/uri').text='model://fw_mini_pro_fairy'
    plugin(w,'ignition-gazebo-sensors-system','Sensors',{'render_engine':'ogre2'})
    boxes=[('north',0,6,12,.2),('south',0,-6,12,.2),('east',6,0,.2,12),('west',-6,0,.2,12),('block_a',3,3,1,1.5),('block_b',-3,2,1.5,.8),('block_c',-2,-3,.7,1.2)]
    for name,x,y,sx,sy in boxes:
        m=child(w,'model',name=name);child(m,'static','true');child(m,'pose',f'{x} {y} 1 0 0 0')
        l=child(m,'link',name='body');geometry(l,'shape','box',{'size':f'{sx} {sy} 2'},'.65 .6 .45 1')
    E.indent(worldtree);worldtree.write(ROOT/'worlds/slam_ground.sdf',encoding='unicode',xml_declaration=True)
    bridges=yaml.safe_load((ROOT/'config/bridge.yaml').read_text())
    bridges.append(dict(ros_topic_name='/rslidar_points',gz_topic_name='/fairy/points',ros_type_name='sensor_msgs/msg/PointCloud2',gz_type_name='ignition.msgs.PointCloudPacked',direction='GZ_TO_ROS'))
    (ROOT/'config/bridge_slam.yaml').write_text(yaml.safe_dump(bridges,sort_keys=False))

if __name__=='__main__':main()
