import os
import math
import time

# (x, y, theta) from your MATLAB list
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

def spawn_markers():
    for i, (x, y, deg) in enumerate(waypoints):
        rad = math.radians(deg)
        
        # Unique ID for each waypoint
        marker_id = i + 100 
        
        # 1. DRAW A SPHERE (The Red Dot)
        # We place it at z=0.05 so it doesn't clip into the floor
        sphere_cmd = f"gz marker -m 'id: {marker_id}, action: ADD, type: SPHERE, " \
                     f"pose: {{position: {{x: {x}, y: {y}, z: 0.05}}}}, " \
                     f"scale: {{x: 0.1, y: 0.1, z: 0.1}}, " \
                     f"material: {{diffuse: {{r: 1, g: 0, b: 0, a: 1}}}}'"
        
        # 2. DRAW AN ARROW (To show orientation)
        # We rotate the arrow to match your theta
        arrow_id = i + 500
        # Quaternions for rotation around Z-axis
        qz = math.sin(rad/2.0)
        qw = math.cos(rad/2.0)
        
        arrow_cmd = f"gz marker -m 'id: {arrow_id}, action: ADD, type: ARROW, " \
                     f"pose: {{position: {{x: {x}, y: {y}, z: 0.1}}, orientation: {{z: {qz}, w: {qw}}}}}, " \
                     f"scale: {{x: 0.4, y: 0.4, z: 0.4}}, " \
                     f"material: {{diffuse: {{r: 0, g: 1, b: 0, a: 1}}}}'"

        os.system(sphere_cmd)
        os.system(arrow_cmd)
        print(f"Spawned waypoint {i} at ({x}, {y})")

if __name__ == "__main__":
    print("Sending markers to Gazebo...")
    spawn_markers()
