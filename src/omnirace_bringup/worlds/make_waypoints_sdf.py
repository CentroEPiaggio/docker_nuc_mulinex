#!/usr/bin/env python3
"""Write a Gazebo waypoint-marker model (same look as waypoints.sdf) from a list of waypoints.

    python3 make_waypoints_sdf.py waypoints_line.sdf "0.5 0 0; 1.0 0 90; 1.5 0 180"

Waypoints are "x y theta_deg" separated by ';'. Markers are visual only (no collisions).
"""
import math
import sys

LINK = """    <link name="waypoint_{i}">
      <pose>{x} {y} 0.01 0 0 {th}</pose>
      <visual name="dot">
        <geometry><sphere><radius>0.05</radius></sphere></geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
        </material>
      </visual>
      <visual name="arrow">
        <pose>0.1 0 0.01 0 1.5707 0</pose>
        <geometry><cylinder><radius>0.01</radius><length>0.2</length></cylinder></geometry>
        <material>
          <ambient>0 1 0 1</ambient>
          <diffuse>0 1 0 1</diffuse>
        </material>
      </visual>
    </link>
"""


def main():
    out, spec = sys.argv[1], sys.argv[2]
    wps = [[float(v) for v in w.split()] for w in spec.split(';') if w.strip()]
    assert all(len(w) == 3 for w in wps), 'each waypoint needs x y theta_deg'
    links = ''.join(LINK.format(i=i, x=x, y=y, th=math.radians(t)) for i, (x, y, t) in enumerate(wps))
    with open(out, 'w') as f:
        f.write('<?xml version="1.0" ?>\n<sdf version="1.6">\n  <model name="path_waypoints">\n'
                '    <static>true</static>\n\n' + links + '  </model>\n</sdf>\n')
    print(f'{out}: {len(wps)} waypoints')


if __name__ == '__main__':
    main()
