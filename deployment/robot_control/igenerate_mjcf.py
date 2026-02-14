import math

def generate_rd_xml(filename="rd_structure.xml"):
    # Vertices of Rhombic Dodecahedron
    # 6 "Axis" vertices (Octahedron tips)
    # 8 "Corner" vertices (Cube corners)
    
    # Scale
    s = 1.0  # Restoring scale to 1.0 (Human scale ~2m) to match Pilot
    z_offset = 1.0 # Height off ground
    
    # Defaults
    th_struct = 0.02 # Structure thickness

    xml_header = """<mujocoinclude>
"""

    # --- 1. GYROSCOPE ASSEMBLY (ROOT) ---
    
    xml_body = ""
    
    # Helper for Ring Geoms
    def get_ring_geoms(radius, color, axis='z', phase=0):
        g = ""
        seg = 32 # Smoother
        gap = 0.1 # Small gap for joints
        # Two Arcs
        for start in [gap, math.pi + gap]:
            for i in range(seg):
                a1 = start + i * (math.pi - 2*gap)/seg + phase
                a2 = start + (i+1) * (math.pi - 2*gap)/seg + phase
                
                c1, s1 = math.cos(a1), math.sin(a1)
                c2, s2 = math.cos(a2), math.sin(a2)
                
                if axis == 'z':
                    p1 = (radius*c1, radius*s1, 0)
                    p2 = (radius*c2, radius*s2, 0)
                elif axis == 'y':
                    p1 = (radius*c1, 0, radius*s1)
                    p2 = (radius*c2, 0, radius*s2)
                elif axis == 'x':
                    p1 = (0, radius*c1, radius*s1)
                    p2 = (0, radius*c2, radius*s2)
                else:
                    p1 = (radius*c1, radius*s1, 0)
                    p2 = (radius*c2, radius*s2, 0)
                    
                g += '        <geom type="capsule" fromto="{:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f}" size="{}" rgba="{}" mass="0.1"/>\n'.format(
                    p1[0], p1[1], p1[2], p2[0], p2[1], p2[2], th_struct, color
                )
        return g

    # --- 1. GYROSCOPE (Central Structure) ---
    # ROOT BODY: Floating Base for the Gyroscope
    xml_body += '    <body name="gyro_root" pos="0 0 {}">\n'.format(z_offset)
    xml_body += '        <freejoint/>\n'
    # --- ROBOT BODY (PILOT) ---
    # Replaces the simple sphere with a humanoid composition
    # Torso (Vertical Capsule)
    xml_body += '        <geom name="torso" type="capsule" fromto="0 0 -0.3 0 0 0.3" size="0.15" mass="40.0" rgba="0.2 0.3 0.4 1" contype="0" conaffinity="0"/>\n'
    # Head (Sphere)
    xml_body += '        <geom name="head" type="sphere" pos="0 0 0.45" size="0.12" mass="5.0" rgba="0.8 0.6 0.4 1" contype="0" conaffinity="0"/>\n'
    # Shoulders (Horizontal Capsule)
    xml_body += '        <geom name="shoulders" type="capsule" fromto="-0.25 0 0.25 0.25 0 0.25" size="0.08" mass="10.0" rgba="0.2 0.3 0.4 1" contype="0" conaffinity="0"/>\n'
    # Arms (Vertical Capsules)
    xml_body += '        <geom name="arm_l" type="capsule" fromto="0.25 0 0.25 0.35 0 -0.2" size="0.06" mass="5.0" rgba="0.2 0.3 0.4 1" contype="0" conaffinity="0"/>\n'
    xml_body += '        <geom name="arm_r" type="capsule" fromto="-0.25 0 0.25 -0.35 0 -0.2" size="0.06" mass="5.0" rgba="0.2 0.3 0.4 1" contype="0" conaffinity="0"/>\n'
    # Hips (Horizontal Capsule)
    xml_body += '        <geom name="hips" type="capsule" fromto="-0.15 0 -0.3 0.15 0 -0.3" size="0.08" mass="10.0" rgba="0.2 0.3 0.4 1" contype="0" conaffinity="0"/>\n'
    # Legs (Vertical Capsules)
    xml_body += '        <geom name="leg_l" type="capsule" fromto="0.15 0 -0.3 0.2 0 -0.8" size="0.07" mass="10.0" rgba="0.2 0.3 0.4 1" contype="0" conaffinity="0"/>\n'
    xml_body += '        <geom name="leg_r" type="capsule" fromto="-0.15 0 -0.3 -0.2 0 -0.8" size="0.07" mass="10.0" rgba="0.2 0.3 0.4 1" contype="0" conaffinity="0"/>\n'
    
    # ---------------------------------------------------------
    # OUTER RING (Gold, YZ Plane) - Y-Axis Joint
    # ---------------------------------------------------------
    xml_body += '        <body name="gyro_outer" pos="0 0 0">\n'
    xml_body += '            <joint name="motor_outer" axis="0 1 0"/>\n' # Y-Axis Pivot
    
    # Visual: Gold Ring (Radius 0.9s to clear pilot)
    xml_body += get_ring_geoms(s*0.9, "1 0.8 0 1", axis='x', phase=math.pi/2)

    # ---------------------------------------------------------
    # MIDDLE RING (Silver, XY Plane) - X-Axis Joint
    # ---------------------------------------------------------
    xml_body += '            <body name="gyro_middle" pos="0 0 0">\n'
    xml_body += '                <joint name="motor_middle" axis="1 0 0"/>\n' # X-Axis Pivot
    
    # Visual: Silver Ring (Radius 0.8s)
    xml_body += get_ring_geoms(s*0.8, "0.8 0.8 0.8 1", axis='z', phase=math.pi/2)

    # ---------------------------------------------------------
    # INNER RING (Bronze, XZ Plane) - Z-Axis Joint
    # ---------------------------------------------------------
    xml_body += '                <body name="gyro_inner" pos="0 0 0">\n'
    xml_body += '                    <joint name="motor_inner" axis="0 0 1"/>\n' # Z-Axis Pivot
    
    # Visual: Bronze Ring (Radius 0.7s)
    xml_body += get_ring_geoms(s*0.7, "0.8 0.5 0.2 1", axis='y')

    # --- TENSEGRITY NODES ATTACHED TO INNER RING ---
    # We attach the Rhombic Dodecahedron Nodes to the Inner Ring so they rotate with it?
    # Or is the Tensegrity the *Outer* Shell? 
    # Usually Tensegrity is the structure. 
    # Let's attach the "Axis" nodes to the Inner Ring to create a cage around the pilot.
    
    # Axis Vertices (Distance 1.2 * s)
    axis_verts = [
        (s, 0, 0), (-s, 0, 0),
        (0, s, 0), (0, -s, 0),
        (0, 0, s), (0, 0, -s)
    ]
    
    # Cube Vertices
    cube_verts = []
    for x in [-1, 1]:
        for y in [-1, 1]:
            for z in [-1, 1]:
                cube_verts.append((x*s*0.6, y*s*0.6, z*s*0.6)) # Slightly smaller cube inside/outside?
    
    # Let's use the Tensegrity Generation Logic from the user's edit, but attached to `gyro_inner`
    
    # Combined List of Nodes
    # NOTE: To make them identifiable as "node_X", we just list them.
    # But they must be children of `gyro_inner` to move with it.
    
    all_nodes = axis_verts + cube_verts
    
    # Edges
    edges = []
    for i in range(len(all_nodes)):
        for j in range(i + 1, len(all_nodes)):
            p1 = all_nodes[i]
            p2 = all_nodes[j]
            dist = math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)
            # Fully connected logic (or user's logic)
            if dist > 0.001: 
                edges.append((p1, p2))

    # Render Nodes
    for i, v in enumerate(all_nodes):
        xml_body += '                    <body name="node_{}" pos="{} {} {}">\n'.format(i, v[0], v[1], v[2])
        xml_body += '                        <geom type="sphere" size="{}" rgba="0.2 0.8 0.2 1" mass="0.01"/>\n'.format(s*0.04)
        xml_body += '                    </body>\n'

    # Render Edges (Attached to Inner Ring frame)
    for index, (start, end) in enumerate(edges):
        xml_body += '                    <geom name="strut_{}" type="capsule" size="{}" fromto="{} {} {} {} {} {}" rgba="0.2 0.6 1 1" mass="0.01"/>\n'.format(
            index, s*0.015, start[0], start[1], start[2], end[0], end[1], end[2]
        )

    xml_body += '                </body>\n' # End Inner
    xml_body += '            </body>\n' # End Middle
    xml_body += '        </body>\n' # End Outer
    xml_body += '    </body>\n' # End Root

    # --- ACTUATORS ---
    xml_actuator = '    <actuator>\n'
    # Gyro Actuators (Torque Control)
    # Range -1 to 1 maps to -200 to 200 Torque.
    xml_actuator += '        <motor name="act_outer"  joint="motor_outer"  gear="200" ctrllimited="true" ctrlrange="-1 1"/>\n'
    xml_actuator += '        <motor name="act_middle" joint="motor_middle" gear="200" ctrllimited="true" ctrlrange="-1 1"/>\n'
    xml_actuator += '        <motor name="act_inner"  joint="motor_inner"  gear="200" ctrllimited="true" ctrlrange="-1 1"/>\n'
    xml_actuator += '    </actuator>\n'

    xml_footer = """</mujocoinclude>
"""
    
    with open(filename, "w") as f:
        f.write(xml_header + xml_body + xml_actuator + xml_footer)
    
    print(f"Generated {filename} with Pilot, Gyro, Tensegrity, and Actuators.")

if __name__ == "__main__":
    generate_rd_xml("public/mujoco/menagerie/unitree_g1/rd_structure.xml")
