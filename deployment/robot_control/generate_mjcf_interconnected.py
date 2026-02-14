import math
import os

def generate_scene_gyro():
    # Paths
    g1_path = "public/mujoco/menagerie/unitree_g1/g1.xml"
    output_path = "public/mujoco/menagerie/unitree_g1/scene_gyro_interconnected.xml"

    # Read G1 XML
    try:
        with open(g1_path, "r") as f:
            g1_content = f.read()
    except FileNotFoundError:
        print(f"Error: Could not find {g1_path}")
        return

    # --- RHOMBIC DODECAHEDRON GENERATION (Preserved) ---
    # Scale
    s = 1.8  # Overall size scale (Human Size: ~2.4m span)
    z_offset = 2.5 # Align with G1 Pelvis Height (Raised to prevent clipping)
    
    # Axis Vertices (Distance 1.0 * s)
    axis_verts = [
        (s, 0, 0), (-s, 0, 0),
        (0, s, 0), (0, -s, 0),
        (0, 0, s), (0, 0, -s)
    ]

    # Defaults to Inline
    d_joint = 'damping="0.5" armature="0.02"'
    d_geom = 'contype="0" conaffinity="0" condim="3" solref="0.02 1" solimp="0.9 0.95 0.001" margin="0.001"'

    # --- 1. GYROSCOPE ASSEMBLY (ROOT) ---
    th_struct = s * 0.015
    xml_body = ""
    
    def get_ring_geoms(radius, color, axis='z', phase=0):
        g = ""
        seg = 16
        gap = 0.2
        for start in [gap, math.pi + gap]:
            for i in range(seg):
                a1 = start + i * (math.pi - 2*gap)/seg + phase
                a2 = start + (i+1) * (math.pi - 2*gap)/seg + phase
                c1, s1 = math.cos(a1), math.sin(a1)
                c2, s2 = math.cos(a2), math.sin(a2)
                
                if axis == 'z': p1, p2 = (radius*c1, radius*s1, 0), (radius*c2, radius*s2, 0)
                elif axis == 'y': p1, p2 = (radius*c1, 0, radius*s1), (radius*c2, 0, radius*s2)
                elif axis == 'x': p1, p2 = (0, radius*c1, radius*s1), (0, radius*c2, radius*s2)
                else: p1, p2 = (radius*c1, radius*s1, 0), (radius*c2, radius*s2, 0)
                    
                g += '        <geom type="capsule" fromto="{:.4f} {:.4f} {:.4f} {:.4f} {:.4f} {:.4f}" size="{}" rgba="{}" mass="1.0" {}/>\n'.format(
                    p1[0], p1[1], p1[2], p2[0], p2[1], p2[2], th_struct, color, d_geom
                )
        return g

    # ROOT BODY
    xml_body += '    <body name="gyro_root" pos="0 0 {}">\n'.format(z_offset)
    xml_body += '        <freejoint/>\n'
    xml_body += '        <geom type="sphere" size="0.2" mass="100.0" rgba="0.5 0.5 0.5 0.3" group="4" {}/>\n'.format(d_geom)
    
    # OUTER RING (Gold, Y-Axis)
    xml_body += '        <body name="gyro_outer" pos="0 0 0">\n'
    # xml_body += '            <joint name="motor_outer" axis="0 1 0" {}/>\n'.format(d_joint)
    xml_body += get_ring_geoms(s*0.55, "1 0.8 0 1", axis='x', phase=math.pi/2)
    # Payload Nodes 2/3
    xml_body += '            <site name="node_2" pos="0 {} 0"/>\n'.format(s)
    xml_body += '            <geom type="sphere" pos="0 {} 0" size="{}" rgba="0.2 0.8 0.2 1" mass="0.1"/>\n'.format(s, s*0.06)
    xml_body += '            <geom type="capsule" fromto="0 {} 0 0 {} 0" size="{}" rgba="1 0.8 0 1" mass="0.01"/>\n'.format(s*0.55, s, th_struct)
    xml_body += '            <site name="node_3" pos="0 {} 0"/>\n'.format(-s)
    xml_body += '            <geom type="sphere" pos="0 {} 0" size="{}" rgba="0.2 0.8 0.2 1" mass="0.1"/>\n'.format(-s, s*0.06)
    xml_body += '            <geom type="capsule" fromto="0 {} 0 0 {} 0" size="{}" rgba="1 0.8 0 1" mass="0.01"/>\n'.format(-s*0.55, -s, th_struct)

    # MIDDLE RING (Silver, X-Axis)
    xml_body += '            <body name="gyro_middle" pos="0 0 0">\n'
    # xml_body += '                <joint name="motor_middle" axis="1 0 0" {}/>\n'.format(d_joint)
    xml_body += get_ring_geoms(s*0.50, "0.8 0.8 0.8 1", axis='z', phase=math.pi/2)
    # Payload Nodes 0/1
    xml_body += '                <site name="node_0" pos="{} 0 0"/>\n'.format(s)
    xml_body += '                <geom type="sphere" pos="{} 0 0" size="{}" rgba="0.2 0.8 0.2 1" mass="0.1"/>\n'.format(s, s*0.06)
    xml_body += '                <geom type="capsule" fromto="{} 0 0 {} 0 0" size="{}" rgba="0.8 0.8 0.8 1" mass="0.01"/>\n'.format(s*0.50, s, th_struct)
    xml_body += '                <site name="node_1" pos="{} 0 0"/>\n'.format(-s)
    xml_body += '                <geom type="sphere" pos="{} 0 0" size="{}" rgba="0.2 0.8 0.2 1" mass="0.1"/>\n'.format(-s, s*0.06)
    xml_body += '                <geom type="capsule" fromto="{} 0 0 {} 0 0" size="{}" rgba="0.8 0.8 0.8 1" mass="0.01"/>\n'.format(-s*0.50, -s, th_struct)

    # INNER RING (Bronze, Z-Axis)
    xml_body += '                <body name="gyro_inner" pos="0 0 0">\n'
    # xml_body += '                    <joint name="motor_inner" axis="0 0 1" {}/>\n'.format(d_joint)
    xml_body += get_ring_geoms(s*0.45, "0.8 0.5 0.2 1", axis='y')
    # Payload Nodes 4/5
    xml_body += '                    <site name="node_4" pos="0 0 {}"/>\n'.format(s)
    xml_body += '                    <geom type="sphere" pos="0 0 {}" size="{}" rgba="0.2 0.8 0.2 1" mass="0.1"/>\n'.format(s, s*0.06)
    xml_body += '                    <geom type="capsule" fromto="0 0 {} 0 0 {}" size="{}" rgba="0.8 0.5 0.2 1" mass="0.01"/>\n'.format(s*0.45, s, th_struct)
    xml_body += '                    <site name="node_5" pos="0 0 {}"/>\n'.format(-s)
    xml_body += '                    <geom type="sphere" pos="0 0 {}" size="{}" rgba="0.2 0.8 0.2 1" mass="0.1"/>\n'.format(-s, s*0.06)
    xml_body += '                    <geom type="capsule" fromto="0 0 {} 0 0 {}" size="{}" rgba="0.8 0.5 0.2 1" mass="0.01"/>\n'.format(-s*0.45, -s, th_struct)
    
    xml_body += '                </body>\n' # End Inner
    xml_body += '            </body>\n' # End Middle
    xml_body += '        </body>\n' # End Outer
    xml_body += '    </body>\n' # End Root

    # --- 2. CUBE NODES (FLOATING BODIES) ---
    cube_coords = []
    for x in [-1, 1]:
        for y in [-1, 1]:
            for z in [-1, 1]:
                cube_coords.append((x * 0.5 * s, y * 0.5 * s, z * 0.5 * s))

    # Floating nodes must be OUTSIDE gyro_root (direct children of worldbody)
    xml_nodes = ""
    for i, v in enumerate(cube_coords):
        idx = 6 + i
        # Position them relative to z_offset
        xml_nodes += '    <body name="node_{}" pos="{} {} {}">\n'.format(idx, v[0], v[1], v[2] + z_offset)
        xml_nodes += '        <freejoint/>\n'
        xml_nodes += '        <site name="node_{}"/>\n'.format(idx)
        xml_nodes += '        <geom type="sphere" size="{}" rgba="0.2 0.8 0.2 1" mass="1.0" {}/>\n'.format(s*0.03, d_geom)
        xml_nodes += '    </body>\n'

    # --- 3. TENDONS, ACTUATORS, EQUALITY ---
    xml_tendon = ""
    xml_actuator = ""
    xml_equality = ""

    # --- 2. DEFINE ALL 14 NODES ---
    # Nodes 0-5: Gyro Rings (Attached to specific bodies)
    # Nodes 6-13: Floating Cube (Attached to their own bodies)
    
    # helper to get init pos dict
    # Note: Rings are at z_offset.
    # Node 0 (Middle): (s,0,0) (local) -> (s, 0, z_offset) (global approx if angle=0)
    # Node 1 (Middle): (-s,0,0)
    # Node 2 (Outer):  (0,s,0)
    # Node 3 (Outer):  (0,-s,0)
    # Node 4 (Inner):  (0,0,s)
    # Node 5 (Inner):  (0,0,-s)
    
    # Mapping Index -> {name, body, init_pos}
    all_nodes = []
    
    # 0, 1: Middle Ring (X-Axis)
    all_nodes.append({'name': 'node_0', 'body': 'gyro_middle', 'pos': (s, 0, z_offset)})
    all_nodes.append({'name': 'node_1', 'body': 'gyro_middle', 'pos': (-s, 0, z_offset)})
    
    # 2, 3: Outer Ring (Y-Axis)
    all_nodes.append({'name': 'node_2', 'body': 'gyro_outer', 'pos': (0, s, z_offset)})
    all_nodes.append({'name': 'node_3', 'body': 'gyro_outer', 'pos': (0, -s, z_offset)})
    
    # 4, 5: Inner Ring (Z-Axis)
    all_nodes.append({'name': 'node_4', 'body': 'gyro_inner', 'pos': (0, 0, s + z_offset)})
    all_nodes.append({'name': 'node_5', 'body': 'gyro_inner', 'pos': (0, 0, -s + z_offset)})
    
    # 6-13: Floating Nodes
    # cube_coords calculated above
    # We need to ensure we match the order of 'xml_nodes' generation loop
    for i, v in enumerate(cube_coords):
        idx = 6 + i
        # The body name for floating node is 'node_{idx}'
        # The pos is v + z_offset
        all_nodes.append({'name': f'node_{idx}', 'body': f'node_{idx}', 'pos': (v[0], v[1], v[2] + z_offset)})

    # --- 3. GENERATE FULLY INTERCONNECTED PISTONS (K14 Graph) ---
    # 14 nodes -> 91 edges
    
    # Disable collision for pistons to avoid center explosion
    d_piston_geom = 'contype="0" conaffinity="0" density="0"' # No Collision, No Inertia (use manually set mass)

    count_piston = 0
    xml_actuator = "" # Reset
    xml_equality = "" # Reset
    
    # Keep Gyro Actuators - REMOVED for static frame
    # xml_actuator += '        <motor name="act_outer"  joint="motor_outer"  gear="1000" ctrllimited="true" ctrlrange="-1 1"/>\n'
    # xml_actuator += '        <motor name="act_middle" joint="motor_middle" gear="1000" ctrllimited="true" ctrlrange="-1 1"/>\n'
    # xml_actuator += '        <motor name="act_inner"  joint="motor_inner"  gear="1000" ctrllimited="true" ctrlrange="-1 1"/>\n'

    for i in range(len(all_nodes)):
        for j in range(i + 1, len(all_nodes)):
            n1 = all_nodes[i]
            n2 = all_nodes[j]
            
            p1 = n1['pos']
            p2 = n2['pos']
            
            # Midpoint and Orientation
            mid_x, mid_y, mid_z = (p1[0]+p2[0])/2, (p1[1]+p2[1])/2, (p1[2]+p2[2])/2
            dx, dy, dz = p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]
            dist = math.sqrt(dx**2 + dy**2 + dz**2) # Fixed Math Domain Error
            if dist < 0.001: continue # Skip overlapping
            
            # Piston Name
            p_name = f"edge_{i}_{j}"
            
            # Lengths
            barrel_len = dist * 0.55
            rod_len = dist * 0.55
            
            # Piston Body (Barrel)
            xml_nodes += f'    <body name="{p_name}_barrel" pos="{mid_x} {mid_y} {mid_z}" zaxis="{dx} {dy} {dz}">\n'
            xml_nodes += '        <freejoint/>\n'
            # Visual Geom only
            xml_nodes += f'        <geom type="capsule" fromto="0 0 {-barrel_len/2} 0 0 {barrel_len/2}" size="{th_struct*1.2}" rgba="0 1 1 0.3" mass="0.5" {d_piston_geom}/>\n'
            
            # Rod Body (Child of Barrel? No, sibling with slide joint? 
            # Or child with slide joint. If child, we don't need freejoint on rod)
            # Standard setup: Barrel has freejoint. Rod is child of Barrel with slide joint.
            # Then we constrain Barrel to N1 and Rod to N2.
            
            xml_nodes += f'        <body name="{p_name}_rod" pos="0 0 0">\n'
            # Increased Stiffnes
            xml_nodes += f'            <joint name="slide_{p_name}" type="slide" axis="0 0 1" limited="false" damping="2000" armature="5.0"/>\n'
            xml_nodes += f'            <geom type="capsule" fromto="0 0 {-rod_len/2} 0 0 {rod_len/2}" size="{th_struct}" rgba="0 1 1 0.6" mass="0.5" {d_piston_geom}/>\n'
            xml_nodes += '        </body>\n'
            xml_nodes += '    </body>\n'
            
            # Constraints (Ball Joints / Connect)
            # Connect Barrel (at negative end) to Node 1
            # Local point of barrel start: (0,0, -dist/2) approx?
            # Actually, we initialized it at Midpoint. 
            # The 'zaxis' aligns local Z with the vector P1->P2.
            # So P1 is at local z = -dist/2. P2 is at local z = +dist/2.
            
            # Anchor 1: Global P1
            # Anchor 2: Global P2
            
            xml_equality += f'        <connect name="conn_{p_name}_1" body1="{p_name}_barrel" body2="{n1["body"]}" anchor="{p1[0]} {p1[1]} {p1[2]}" solref="0.02 1"/>\n'
            
            # Rod is connected to P2. Rod is usually centered at 000 of parent (unless offset).
            # We placed rod at 000 of barrel.
            # So Rod is also at Midpoint.
            # P2 is at local z = +dist/2 relative to Rod?
            # We constrain Rod Body to N2.
            xml_equality += f'        <connect name="conn_{p_name}_2" body1="{p_name}_rod" body2="{n2["body"]}" anchor="{p2[0]} {p2[1]} {p2[2]}" solref="0.02 1"/>\n'
            
            # Actuator
            # Use high KP for strong connection
            xml_actuator += f'        <position name="act_{p_name}" joint="slide_{p_name}" kp="5000" kv="500" ctrllimited="true" ctrlrange="-0.5 0.5"/>\n'
            
            count_piston += 1

    # Robot Attachment Weld
    xml_equality += '        <weld name="attach_robot" body1="gyro_inner" body2="pelvis" relpose="0 0 0 1 0 0 0"/>\n'

    # --- INJECTION ---
    final_xml = g1_content
    
    # Assets (Floor Material)
    floor_asset = """
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
    """
    if '</asset>' in final_xml:
        final_xml = final_xml.replace('</asset>', floor_asset + '\n  </asset>')
    else:
        final_xml = final_xml.replace('<worldbody>', f'<asset>{floor_asset}</asset>\n<worldbody>')

    # Worldbody: Floor + Gyro + Nodes
    floor_xml = '<geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>'
    final_xml = final_xml.replace('<worldbody>', f'<worldbody>\n    {floor_xml}\n    {xml_body}\n    {xml_nodes}')
    
    # Actuators
    if '</actuator>' in final_xml:
        final_xml = final_xml.replace('</actuator>', xml_actuator + '\n  </actuator>')
    else:
        final_xml = final_xml.replace('</mujoco>', f'<actuator>{xml_actuator}</actuator>\n</mujoco>')
        
    # Tendons
    if xml_tendon:
        if '</tendon>' in final_xml:
             final_xml = final_xml.replace('</tendon>', xml_tendon + '\n  </tendon>')
        else:
             final_xml = final_xml.replace('</mujoco>', f'<tendon>{xml_tendon}</tendon>\n</mujoco>')

    # Equality
    if xml_equality:
        if '</equality>' in final_xml:
             final_xml = final_xml.replace('</equality>', xml_equality + '\n  </equality>')
        else:
             final_xml = final_xml.replace('</mujoco>', f'<equality>{xml_equality}</equality>\n</mujoco>')

    with open(output_path, "w") as f:
        f.write(final_xml)

    print(f"Generated Merged Scene: {output_path}")

if __name__ == "__main__":
    generate_scene_gyro()
