import math
import os

def generate_scene_clean():
    # Paths
    g1_path = "public/mujoco/menagerie/unitree_g1/g1.xml"
    output_path = "public/mujoco/menagerie/unitree_g1/scene_puppet.xml"

    # Read G1 XML
    try:
        with open(g1_path, "r") as f:
            g1_content = f.read()
    except FileNotFoundError:
        print(f"Error: Could not find {g1_path}")
        return

    # --- PUPPET MODE: KEEP G1 INTERNAL MOTORS ---
    # We want the robot to drive the external fascia.
    # Keep the <actuator> block from the G1 XML.
    # import re
    # g1_content = re.sub(r'<actuator>.*?</actuator>', '', g1_content, flags=re.DOTALL)
    print("Puppet Mode: Keep G1 Internal Actuators.")

    # --- RHOMBIC DODECAHEDRON GENERATION ---
    # Scale
    s = 1.8  # Overall size scale
    z_offset = 1.0 # Align with G1 Pelvis Height
    
    # Axis Vertices (Distance 1.0 * s)
    axis_verts = [
        (s, 0, 0), (-s, 0, 0),
        (0, s, 0), (0, -s, 0),
        (0, 0, s), (0, 0, -s)
    ]

    # STABILITY SETTINGS (High Damping/Inertia)
    # Damping 10.0 -> 1.0 (Less resistance)
    d_joint = 'damping="1.0" armature="1.0"'
    # Disable Collisions (Phantom/Visual Only) - Static Frame which Floats (Gravity Comp)
    d_geom = 'contype="0" conaffinity="0" solref="0.005 1" solimp="0.9 0.95 0.001" margin="0.001"'
    th_struct = s * 0.015

    # Storage for generated XML parts
    xml_nodes = ""
    xml_equality = ""
    xml_actuator = ""
    xml_tendon = ""
    xml_contact = ""

    # Helper: Add Piston (Linear Actuator)
    def add_piston(n1_idx, n2_idx, p1, p2, name_suffix, color):
        nonlocal xml_nodes, xml_equality, xml_actuator
        p_name = f"piston_{name_suffix}"
        mid_x, mid_y, mid_z = (p1[0]+p2[0])/2, (p1[1]+p2[1])/2, (p1[2]+p2[2])/2 + z_offset
        dx, dy, dz = p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        h_len = dist * 0.6
        
        # Double Acting Piston Dimensions
        # Barrel: Central, length 40% of dist. (-0.2 to 0.2)
        # Rods: Extend from barrel to nodes.
        # At Neutral (q=0):
        #   Rod A visual: -0.1 (inside) to -0.5 (anchor).
        #   Rod B visual:  0.1 (inside) to  0.5 (anchor).
        
        barrel_half = dist * 0.2
        barrel_inner_reach = dist * 0.1 # Visual start of rod inside barrel
        node_reach = dist * 0.5
        
        # Per-Rod Ranges
        # Compression: Rod can retract into barrel.
        #   Max Retract: until anchor hits barrel end (-0.2)? Or deeper.
        #   Let's allow retracting until anchor is at -0.25 (dist/4).
        #   Current Anchor (-0.5) -> Target (-0.25) implies q = -0.25.
        limit_comp = -dist * 0.25
        
        # Extension: Rod can extend out.
        #   Max Extend: until overlap lost.
        #   Current Inner (-0.1) -> Target (-0.2) (Edge of barrel). q = 0.1.
        limit_ext = dist * 0.1
        
        # Piston Barrel (Central Housing - Dark Grey)
        xml_nodes += f'    <body name="{p_name}_barrel" pos="{mid_x} {mid_y} {mid_z}" zaxis="{dx} {dy} {dz}" gravcomp="1">\n'
        xml_nodes += '        <freejoint/>\n'
        xml_nodes += f'        <geom type="cylinder" fromto="0 0 {-barrel_half} 0 0 {barrel_half}" size="{th_struct*2.0}" rgba="0.2 0.2 0.2 1" mass="1.0" {d_geom}/>\n'
        # No anchor on barrel, it floats between rods.
        
        # Rod A (Left/Bottom - Negative Z)
        xml_nodes += f'        <body name="{p_name}_rod_a" pos="0 0 0" gravcomp="1">\n'
        # Axis -1 means positive q moves towards negative Z (Extension away from center)
        xml_nodes += f'            <joint name="slide_{p_name}_a" type="slide" axis="0 0 -1" limited="true" range="{limit_comp} {limit_ext}" {d_joint}/>\n'
        xml_nodes += f'            <geom type="cylinder" fromto="0 0 {-barrel_inner_reach} 0 0 {-node_reach}" size="{th_struct}" rgba="{color}" mass="0.5" {d_geom}/>\n'
        xml_nodes += f'            <site name="{p_name}_anchor_a" pos="0 0 {-node_reach}"/>\n'
        xml_nodes += '        </body>\n'

        # Rod B (Right/Top - Positive Z)
        xml_nodes += f'        <body name="{p_name}_rod_b" pos="0 0 0" gravcomp="1">\n'
        # Axis +1 means positive q moves towards positive Z (Extension away from center)
        xml_nodes += f'            <joint name="slide_{p_name}_b" type="slide" axis="0 0 1" limited="true" range="{limit_comp} {limit_ext}" {d_joint}/>\n'
        xml_nodes += f'            <geom type="cylinder" fromto="0 0 {barrel_inner_reach} 0 0 {node_reach}" size="{th_struct}" rgba="{color}" mass="0.5" {d_geom}/>\n'
        xml_nodes += f'            <site name="{p_name}_anchor_b" pos="0 0 {node_reach}"/>\n'
        xml_nodes += '        </body>\n'

        xml_nodes += '    </body>\n'
        
        # Weld Constraints to Nodes (Rod A -> N1, Rod B -> N2)
        xml_equality += f'        <weld name="weld_{p_name}_a" body1="{p_name}_rod_a" body2="node_{n1_idx}"/>\n'
        xml_equality += f'        <weld name="weld_{p_name}_b" body1="{p_name}_rod_b" body2="node_{n2_idx}"/>\n'
        
        # Symmetry Constraint (Keep Barrel Centered)
        # Forces slide_a == slide_b, eliminating the floating DOF of the barrel.
        xml_equality += f'        <joint name="eq_{p_name}" joint1="slide_{p_name}_a" joint2="slide_{p_name}_b" polycoef="0 1 0 0 0"/>\n'
        
        # Actuators (Dual - though now coupled, redundant but keeps control authority)
        # Higher Gain 1000->10000
        xml_actuator += f'        <position name="act_{p_name}_a" joint="slide_{p_name}_a" kp="10000" ctrllimited="true" ctrlrange="{limit_comp} {limit_ext}"/>\n'
        # xml_actuator += f'        <position name="act_{p_name}_b" joint="slide_{p_name}_b" kp="1000" ctrllimited="true" ctrlrange="{limit_comp} {limit_ext}"/>\n'

    # --- 1. OCTAHEDRON NODES (0-5) ---
    print(f"Generating Octahedron Nodes: {len(axis_verts)}")
    for i, v in enumerate(axis_verts):
        xml_nodes += '    <body name="node_{}" pos="{} {} {}" gravcomp="1">\n'.format(i, v[0], v[1], v[2] + z_offset)
        xml_nodes += '        <freejoint/>\n' # Re-enabled (Kinematic)
        xml_nodes += '        <site name="node_{}"/>\n'.format(i)
        xml_nodes += '        <geom type="sphere" size="{}" rgba="0.8 0.5 0.2 1" mass="5.0" {}/>\n'.format(s*0.04, d_geom) # Mass 5.0
        xml_nodes += '    </body>\n'

    # --- 2. CUBE NODES (6-13) ---
    cube_coords = []
    for x in [-1, 1]:
        for y in [-1, 1]:
            for z in [-1, 1]:
                cube_coords.append((x * 0.5 * s, y * 0.5 * s, z * 0.5 * s))

    print(f"Generating Cube Nodes: {len(cube_coords)}")
    for i, v in enumerate(cube_coords):
        idx = 6 + i
        xml_nodes += '    <body name="node_{}" pos="{} {} {}" gravcomp="1">\n'.format(idx, v[0], v[1], v[2] + z_offset)
        xml_nodes += '        <freejoint/>\n' # Re-enabled (Kinematic)
        xml_nodes += '        <site name="node_{}"/>\n'.format(idx)
        xml_nodes += '        <geom type="sphere" size="{}" rgba="0.2 0.8 0.2 1" mass="5.0" {}/>\n'.format(s*0.03, d_geom)
        xml_nodes += '    </body>\n'

    # --- JOINT MAPPING ---
    # 12 Leg Joints -> Octahedron Pistons
    leg_joints = [
        "left_hip_pitch_joint", "left_hip_roll_joint", "left_hip_yaw_joint", 
        "left_knee_joint", "left_ankle_pitch_joint", "left_ankle_roll_joint",
        "right_hip_pitch_joint", "right_hip_roll_joint", "right_hip_yaw_joint", 
        "right_knee_joint", "right_ankle_pitch_joint", "right_ankle_roll_joint"
    ]
    
    # 12 Arm Joints -> Cube Pistons
    arm_joints = [
        "left_shoulder_pitch_joint", "left_shoulder_roll_joint", "left_shoulder_yaw_joint",
        "left_elbow_joint", "left_wrist_roll_joint", "left_wrist_pitch_joint",
        "right_shoulder_pitch_joint", "right_shoulder_roll_joint", "right_shoulder_yaw_joint",
        "right_elbow_joint", "right_wrist_roll_joint", "right_wrist_pitch_joint"
    ]

    # --- 3. ACTUATORS (PISTONS) ---
    
    # A. Octahedron Pistons (12 Edges) -> LEGS
    # Connects Nodes 0-5
    oct_edge_len = s * math.sqrt(2)
    oct_tol = 0.05 * s
    count_oct = 0
    
    for i in range(len(axis_verts)):
        for j in range(i + 1, len(axis_verts)):
            dist = math.sqrt(sum((axis_verts[i][k]-axis_verts[j][k])**2 for k in range(3)))
            if abs(dist - oct_edge_len) < oct_tol:
                if count_oct < len(leg_joints):
                    j_name = leg_joints[count_oct]
                    add_piston(i, j, axis_verts[i], axis_verts[j], f"{j_name}_piston", "0.6 0.6 0.6 1")
                else:
                    # Fallback if logic fails (shouldn't if 12 edges)
                    add_piston(i, j, axis_verts[i], axis_verts[j], f"oct_{count_oct}", "0.6 0.6 0.6 1")
                count_oct += 1

    # B. Cube Pistons (12 Edges) -> ARMS
    # Connects Nodes 6-13
    cube_edge_len = s
    cube_tol = 0.05 * s
    count_cube = 0
    
    for i in range(len(cube_coords)):
        for j in range(i + 1, len(cube_coords)):
            dist = math.sqrt(sum((cube_coords[i][k]-cube_coords[j][k])**2 for k in range(3)))
            if abs(dist - cube_edge_len) < cube_tol:
                if count_cube < len(arm_joints):
                    j_name = arm_joints[count_cube]
                    add_piston(6+i, 6+j, cube_coords[i], cube_coords[j], f"{j_name}_piston", "0.6 0.6 0.6 1")
                else:
                    add_piston(6+i, 6+j, cube_coords[i], cube_coords[j], f"cube_{count_cube}", "0.6 0.6 0.6 1")
                count_cube += 1

    # --- 4. TENDONS (Rhombic Springs) ---
    # Connect Octahedron <-> Cube
    # Keep as passive springs
    rd_edge_len = math.sqrt(0.75) * s
    rd_tol = 0.05 * s
    count_spring = 0
    
    for i in range(len(axis_verts)):
        for j in range(len(cube_coords)):
            dist = math.sqrt(sum((axis_verts[i][k]-cube_coords[j][k])**2 for k in range(3)))
            if abs(dist - rd_edge_len) < rd_tol:
                t_name = f"spring_{count_spring}"
                xml_tendon += f'        <spatial name="{t_name}" stiffness="100" damping="10" width="0.01" rgba="1 0.4 0.2 1">\n'
                xml_tendon += f'            <site site="node_{i}"/>\n'
                xml_tendon += f'            <site site="node_{6+j}"/>\n'
                xml_tendon += '        </spatial>\n'
                count_spring += 1



    # --- 5. VISUAL VECTOR EQUILIBRIUM (Cuboctahedron) ---
    # 12 Vertices: Permutations of (+/- k, +/- k, 0).
    # Radius from origin: sqrt(k^2 + k^2) = k * sqrt(2).
    # Let's set radius R = s * 0.5 (Inside the Cube component).
    # k = R / sqrt(2).
    
    ve_radius = s * 0.7071 # Approx s / sqrt(2)
    k = ve_radius / math.sqrt(2)
    
    ve_verts = []
    # Generate 12 Vertices
    # XY Plane: (+/-k, +/-k, 0) -> 4
    for x in [-1, 1]:
        for y in [-1, 1]:
            ve_verts.append((x*k, y*k, 0))
    # YZ Plane: (0, +/-k, +/-k) -> 4
    for y in [-1, 1]:
        for z in [-1, 1]:
            ve_verts.append((0, y*k, z*k))
    # XZ Plane: (+/-k, 0, +/-k) -> 4
    for x in [-1, 1]:
        for z in [-1, 1]:
            ve_verts.append((x*k, 0, z*k))
            
    print(f"Generating Vector Equilibrium Nodes: {len(ve_verts)}")
    
    for i, v in enumerate(ve_verts):
        idx = 14 + i
        xml_nodes += '    <body name="node_{}" pos="{} {} {}" gravcomp="1">\n'.format(idx, v[0], v[1], v[2] + z_offset) # Named node_X for Weld
        xml_nodes += '        <freejoint/>\n'
        xml_nodes += '        <site name="node_{}"/>\n'.format(idx) # Site for weld check? Actually Weld uses Body Name usually? 
        # Wait, add_piston uses "weld ... body2='node_{n2_idx}'". So body name must be node_{idx}.
        xml_nodes += '        <geom type="sphere" size="{}" rgba="0.2 1.0 0.2 0.6" contype="0" conaffinity="0"/>\n'.format(s*0.02)
        xml_nodes += '    </body>\n'

    # VE Edges (Length = Radius = ve_radius)
    # Check all pairs. If dist ~ ve_radius, draw edge as PISTON.
    ve_edge_tol = ve_radius * 0.1
    count_ve_edge = 0
    
    # VE Nodes start indices:
    # 0-5: Octa (6)
    # 6-13: Cube (8)
    # 14-25: VE (12)
    ve_start_idx = 14
    
    for i in range(len(ve_verts)):
        for j in range(i + 1, len(ve_verts)):
            dist = math.sqrt(sum((ve_verts[i][x]-ve_verts[j][x])**2 for x in range(3)))
            if abs(dist - ve_radius) < ve_edge_tol:
                # Add Piston instead of Cylinder
                # Indices in global list: ve_start_idx + i, ve_start_idx + j
                idx1 = ve_start_idx + i
                idx2 = ve_start_idx + j
                
                # Name: ve_piston_0, etc.
                # Color: Green (0.2 1.0 0.2 1)
                add_piston(idx1, idx2, ve_verts[i], ve_verts[j], f"ve_{count_ve_edge}", "0.2 1.0 0.2 1")
                count_ve_edge += 1

    # --- 5. ROBOT ATTACHMENT ---
    # Create a static center anchor for the robot
    xml_nodes += '    <body name="center_anchor" pos="0 0 {}">\n'.format(z_offset)
    xml_nodes += '        <site name="center_site"/>\n'
    # xml_nodes += '        <geom type="sphere" size="0.05" rgba="1 0 0 0.5" mass="0.01" {}/>\n'.format(d_geom) # Visual debug
    xml_nodes += '    </body>\n'

    # Weld Robot to Center Anchor (Geometric Center)
    xml_equality += '        <weld name="attach_robot" body1="center_anchor" body2="pelvis" relpose="0 0 0 1 0 0 0"/>\n'

    # --- INJECTION ---
    final_xml = g1_content
    
    # Assets
    floor_asset = """
    <texture type="2d" name="groundplane" builtin="checker" mark="edge" rgb1="0.2 0.3 0.4" rgb2="0.1 0.2 0.3" markrgb="0.8 0.8 0.8" width="300" height="300"/>
    <material name="groundplane" texture="groundplane" texuniform="true" texrepeat="5 5" reflectance="0.2"/>
    """
    if '</asset>' in final_xml:
        final_xml = final_xml.replace('</asset>', floor_asset + '\n  </asset>')
    else:
        final_xml = final_xml.replace('<worldbody>', f'<asset>{floor_asset}</asset>\n<worldbody>')

    # Worldbody
    floor_xml = '<geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>'
    final_xml = final_xml.replace('<worldbody>', f'<worldbody>\n    {floor_xml}\n    {xml_nodes}')
    
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
             
    # Contacts
    if xml_contact:
        if '</contact>' in final_xml:
             final_xml = final_xml.replace('</contact>', xml_contact + '\n  </contact>')
        else:
             final_xml = final_xml.replace('</mujoco>', f'<contact>{xml_contact}</contact>\n</mujoco>')

    with open(output_path, "w") as f:
        f.write(final_xml)

    print(f"Generated Merged Scene: {output_path}")

if __name__ == "__main__":
    generate_scene_clean()
