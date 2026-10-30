#!/usr/bin/env python3
"""Rebuild the portable SDF and bridge config. SI units, estimated inertias."""
from pathlib import Path
import xml.etree.ElementTree as E
import yaml

ROOT = Path(__file__).resolve().parents[1]
WHEELS = [('front_left',.18,.14),('front_right',.18,-.14),('rear_left',-.18,.14),('rear_right',-.18,-.14)]


def child(parent, tag, text=None, **attrs):
    e = E.SubElement(parent, tag, attrs)
    if text is not None:
        e.text = str(text)
    return e


def inertia(link, mass, diagonal):
    i = child(link,'inertial')
    child(i,'mass',mass)
    tensor = child(i,'inertia')
    for k,v in zip(('ixx','iyy','izz'),diagonal):
        child(tensor,k,v)
    for k in ('ixy','ixz','iyz'):
        child(tensor,k,0)


def geometry(link, name, shape, values, color, pose='0 0 0 0 0 0'):
    for kind in ('collision','visual'):
        obj = child(link,kind,name=name)
        child(obj,'pose',pose)
        geo = child(child(obj,'geometry'),shape)
        for key,value in values.items():
            child(geo,key,value)
        if kind == 'visual':
            mat = child(obj,'material')
            child(mat,'ambient',color)
            child(mat,'diffuse',color)
        else:
            ode = child(child(child(obj,'surface'),'friction'),'ode')
            child(ode,'mu',1.0)
            child(ode,'mu2',1.0)


def plugin(model, filename, name, params):
    p = child(model,'plugin',filename=filename,name='ignition::gazebo::systems::'+name)
    for key,value in params.items():
        child(p,key,value)


def main():
    sdf = E.Element('sdf',version='1.8')
    model = child(sdf,'model',name='fw_mini_pro')
    child(model,'self_collide','false')
    base = child(model,'link',name='base_link')
    # Model origin is chassis centre projected onto wheel axle height.
    # 22 kg base + four 0.5 kg knuckles + four 1.5 kg tires = 30 kg.
    ixyz = tuple(22*(a*a+b*b)/12 for a,b in ((.24,.16),(.565,.16),(.565,.24)))
    inertia(base,22,ixyz)
    child(base.find('inertial'),'pose','0 0 0.08 0 0 0')
    geometry(base,'chassis','box',{'size':'.565 .24 .16'},'.15 .19 .22 1','0 0 .08 0 0 0')
    # Visual deck only: vendor overall width and height, tires remain separate.
    deck = child(base,'visual',name='deck')
    child(deck,'pose','0 0 .17 0 0 0')
    child(child(child(deck,'geometry'),'box'),'size','.52 .38 .03')
    mat = child(deck,'material')
    child(mat,'diffuse','.65 .68 .7 1')
    for name,x,y in WHEELS:
        knuckle = child(model,'link',name=name+'_knuckle')
        child(knuckle,'pose',f'{x} {y} 0 0 0 0')
        inertia(knuckle,.5,(.0003,.0003,.0003))
        wheel = child(model,'link',name=name+'_wheel')
        child(wheel,'pose',f'{x} {y} 0 0 0 0')
        radial = 1.5*(3*.07**2+.045**2)/12
        inertia(wheel,1.5,(radial,1.5*.07**2/2,radial))
        geometry(wheel,'tire','cylinder',{'radius':.07,'length':.045},'.055 .055 .06 1','0 0 0 1.5707963267948966 0 0')
        for suffix,parent,kid,axis in [('steer','base_link',name+'_knuckle','0 0 1'),('wheel',name+'_knuckle',name+'_wheel','0 1 0')]:
            j = child(model,'joint',name=name+'_'+suffix+'_joint',type='revolute')
            child(j,'parent',parent)
            child(j,'child',kid)
            a = child(j,'axis')
            child(a,'xyz',axis)
            limits = child(a,'limit')
            child(limits,'lower',-1e16)
            child(limits,'upper',1e16)
            child(limits,'effort',12 if suffix=='steer' else 8)
            child(limits,'velocity',3 if suffix=='steer' else 22)
            child(child(a,'dynamics'),'damping',.05)
        plugin(model,'ignition-gazebo-joint-position-controller-system','JointPositionController',{
            'joint_name':name+'_steer_joint','topic':'/sim/'+name+'/steer',
            'p_gain':30,'i_gain':0,'d_gain':2,'cmd_max':12,'cmd_min':-12})
        plugin(model,'ignition-gazebo-joint-controller-system','JointController',{
            'joint_name':name+'_wheel_joint','topic':'/sim/'+name+'/wheel',
            'use_force_commands':'true','p_gain':1.0,'i_gain':0,'d_gain':0,'cmd_max':8,'cmd_min':-8})
    plugin(model,'ignition-gazebo-odometry-publisher-system','OdometryPublisher',{
        'odom_frame':'odom','robot_base_frame':'base_link','odom_publish_frequency':50,
        'odom_topic':'/sim/odometry','dimensions':3})
    plugin(model,'ignition-gazebo-joint-state-publisher-system','JointStatePublisher',{'topic':'/sim/joint_states'})
    # Use separated vendor visuals when available; primitives stay as collision.
    for link in model.findall('link'):
        stem = 'chassis' if link.get('name') == 'base_link' else link.get('name')
        mesh = ROOT/'models/fw_mini_pro/meshes'/f'{stem}.stl'
        if mesh.exists():
            for visual in list(link.findall('visual')):
                link.remove(visual)
            visual = child(link,'visual',name='vendor_'+stem)
            child(child(child(visual,'geometry'),'mesh'),'uri',f'model://fw_mini_pro/meshes/{stem}.stl')
            mat = child(visual,'material')
            color = '.12 .12 .13 1' if stem.endswith('_wheel') else '.65 .68 .7 1'
            child(mat,'ambient',color)
            child(mat,'diffuse',color)
    E.indent(sdf)
    E.ElementTree(sdf).write(ROOT/'models/fw_mini_pro/model.sdf',encoding='unicode',xml_declaration=True)
    bridges = []
    for ros,gz,rt,gt in [('/clock','/clock','rosgraph_msgs/msg/Clock','ignition.msgs.Clock'),('/sim/odometry','/sim/odometry','nav_msgs/msg/Odometry','ignition.msgs.Odometry'),('/joint_states','/sim/joint_states','sensor_msgs/msg/JointState','ignition.msgs.Model')]:
        bridges.append(dict(ros_topic_name=ros,gz_topic_name=gz,ros_type_name=rt,gz_type_name=gt,direction='GZ_TO_ROS'))
    for name,_,_ in WHEELS:
        for joint in ('steer','wheel'):
            bridges.append(dict(ros_topic_name=f'/sim/{name}/{joint}',gz_topic_name=f'/sim/{name}/{joint}',ros_type_name='std_msgs/msg/Float64',gz_type_name='ignition.msgs.Double',direction='ROS_TO_GZ'))
    (ROOT/'config/bridge.yaml').write_text(yaml.safe_dump(bridges,sort_keys=False))


if __name__ == '__main__':
    main()
