import math

waypoints = [
    (1.00, 1.00, -90), (1.25, 0.50, -45), (1.75, 0.25, 0), (2.25, 0.50, 45),
    (2.50, 1.00, 60), (2.75, 1.50, 45), (3.25, 1.75, 0), (3.75, 1.50, -45),
    (4.00, 1.00, -90), (3.75, 0.50, -120), (3.25, 0.25, 180), (2.75, 0.50, 120),
    (2.50, 1.00, 120), (2.25, 1.50, 135), (1.75, 1.75, 180), (1.25, 1.50, -120),
    (1.00, 1.00, -90), (1.00, 0.50, 90), (0.75, 2.00, 60), (1.25, 2.50, 0),
    (2.00, 2.00, 45), (2.00, 2.50, -60), (2.75, 2.50, -180), (3.50, 2.25, 150),
    (4.00, 1.75, 135), (4.25, 1.25, 90), (5.00, 0.75, 0), (4.00, 0.75, 0),
    (3.50, 0.75, 180), (3.00, 1.00, 150), (2.50, 1.25, -165), (1.50, 1.25, 90),
    (0.00, 2.00, 0)
]

sdf_content = """<?xml version="1.0" ?>
<sdf version="1.6">
  <model name="path_waypoints">
    <static>true</static>
"""

for i, (x, y, deg) in enumerate(waypoints):
    rad = math.radians(deg)
    # Visual for the "Dot" (Sphere)
    sdf_content += f"""
    <link name="waypoint_{i}">
      <pose>{x} {y} 0.01 0 0 {rad}</pose>
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
    </link>"""

sdf_content += "\n  </model>\n</sdf>"

with open("waypoints.sdf", "w") as f:
    f.write(sdf_content)

print("Created waypoints.sdf successfully!")
