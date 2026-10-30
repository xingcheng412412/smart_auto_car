#!/usr/bin/env python3
"""Split vendor ROS STL into chassis, knuckles and wheels for visualization.

Input: FW-mini-pro-description-ros2/meshes/FW-mini-pro.STL from supplied RAR.
Uses disconnected components, not boolean slicing; collision remains primitive.
"""
import argparse
from collections import defaultdict
from pathlib import Path
import struct
import numpy as np

DTYPE = np.dtype([('normal','<f4',3),('vertices','<f4',(3,3)),('attribute','<u2')])
WHEELS = [('front_left',.18,.14),('front_right',.18,-.14),('rear_left',-.18,.14),('rear_right',-.18,-.14)]


def write_stl(path, vertices):
    data = np.zeros(len(vertices),dtype=DTYPE)
    data['vertices'] = vertices
    normals = np.cross(vertices[:,1]-vertices[:,0],vertices[:,2]-vertices[:,0])
    lengths = np.linalg.norm(normals,axis=1)
    lengths[lengths == 0] = 1
    data['normal'] = normals/lengths[:,None]
    with path.open('wb') as stream:
        stream.write(b'FW-mini-pro vendor mesh, component split'.ljust(80,b'\0'))
        stream.write(struct.pack('<I',len(data)))
        stream.write(data.tobytes())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('stl',type=Path)
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parents[1]/'models/fw_mini_pro/meshes')
    args = parser.parse_args()
    data = np.fromfile(args.stl,dtype=DTYPE,offset=84)
    # Vendor URDF rotates STL by RPY (pi/2,0,-pi/2). Align actual
    # wheel centres to ±180 mm / ±140 mm and axle height to z=0.
    raw = data['vertices'].astype(float)
    v = np.stack((.28335-raw[:,:,2],.18945-raw[:,:,0],raw[:,:,1]-.07605),axis=2)
    parents = list(range(len(v)))
    def root(i):
        while parents[i] != i:
            parents[i] = parents[parents[i]]
            i = parents[i]
        return i
    seen = {}
    for i,triangle in enumerate(v):
        for point in triangle:
            key = tuple(np.round(point,6))
            if key in seen:
                parents[root(i)] = root(seen[key])
            else:
                seen[key] = i
    groups = defaultdict(list)
    for i in range(len(v)):
        groups[root(i)].append(i)
    buckets = defaultdict(list)
    for indices in groups.values():
        points = v[indices].reshape(-1,3)
        low,high = points.min(0),points.max(0)
        target = 'chassis'
        origin = np.zeros(3)
        for name,x,y in WHEELS:
            # Whole disconnected assemblies within one wheel envelope.
            if (low[0] >= x-.077 and high[0] <= x+.077 and
                    low[1] >= y-.07 and high[1] <= y+.07 and high[2] <= .13):
                target = name+('_wheel' if high[2] < .073 else '_knuckle')
                origin = np.array([x,y,0])
                break
        buckets[target].append(v[indices]-origin)
    args.output.mkdir(parents=True,exist_ok=True)
    assert sum(sum(len(a) for a in arrays) for arrays in buckets.values()) == len(v)
    for name,arrays in buckets.items():
        mesh = np.concatenate(arrays)
        write_stl(args.output/(name+'.stl'),mesh)
        print(name,len(mesh),'triangles')
    for name,_,_ in WHEELS:
        if name+'_wheel' not in buckets or name+'_knuckle' not in buckets:
            raise RuntimeError('Unexpected vendor STL: missing wheel or knuckle components')


if __name__ == '__main__':
    main()
