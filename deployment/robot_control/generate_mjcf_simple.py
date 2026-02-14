import math
import os

def generate_scene_simple():
    # Paths
    g1_path = "public/mujoco/menagerie/unitree_g1/g1.xml"
    output_path = "public/mujoco/menagerie/unitree_g1/scene_gyro_simple.xml"

    # Read G1 XML
    try:
        with open(g1_path, "r") as f:
            g1_content = f.read()
    except FileNotFoundError:
        print(f"Error: Could not find {g1_path}")
        return

    # --- PARAMETERS ---
    s = 1.8  # Scale
    z_offset = 2.5 # Height
    
    # Physics Parameters (High Stability)
    # solref: time constant, damping ratio. 0.02 is slow/stable.
    d_geom = 'contype="0" conaffinity="0" condim="3" solref="0.02 1" solimp="0.9 0.95 0.001" margin="0.001"'
    d_piston_geom = 'contype="0" conaffinity="0" group="1" density="0"' 
    
    mass_node = 2.0
    mass_piston = 1.0
    
    # Actuator Gains
    # KP: Stiffness. KV: Damping.
    # Lower KP + High KV = Syrupy, stable movement.
    gain_kp = "2000" 
    gain_kv = "1000"
    
    # Joint Damping (Internal friction of pistons)
    joint_damping = "1000" 
    joint_armature = "1.0" # Inertia

    # --- VERTICES (Rhombic Dodecahedron = Cube + Octahedron) ---
    nodes = []
    
    # 1. Octahedron (6 Axis Vertices) - Indexes 0-5
    axis_verts = [
        (s, 0, 0), (-s, 0, 0),
        (0, s, 0), (0, -s, 0),
        (0, 0, s), (0, 0, -s)
    ]
    for i, v in enumerate(axis_verts):
        nodes.append({'name': f'node_oct_{i}', 'pos': v, 'type': 'octahedron'})

    # 2. Cube (8 Corner Vertices) - Indexes 6-13
    cube_verts = []
    for x in [-1, 1]:
        for y in [-1, 1]:
            for z in [-1, 1]:
                cube_verts.append((x * 0.5 * s, y * 0.5 * s, z * 0.5 * s))
    
    for i, v in enumerate(cube_verts):
        nodes.append({'name': f'node_cube_{i}', 'pos': v, 'type': 'cube'})

    # --- GENERATE XML ---
    xml_body = ""
    xml_equality = ""
    xml_actuator = ""
    xml_tendon = ""
    
    # 1. Generate Bodies for Nodes
    for n in nodes:
        px, py, pz = n['pos']
        # Shift only Z by z_offset (Robot is at 0,0,0 relative to this structure center?)
        # Robot is at global 0,0,0 initially? No, robot usually spawns at some height.
        # We will spawn the structure at z_offset, and the Robot at z_offset.
        
        xml_body += f'    <body name="{n["name"]}" pos="{px} {py} {pz + z_offset}">\n'
        xml_body += '        <freejoint/>\n'
        xml_body += f'        <geom type="sphere" size="{s*0.04}" rgba="0.2 0.8 0.2 1" mass="{mass_node}" {d_geom}/>\n'
        xml_body += f'        <site name="site_{n["name"]}" pos="0 0 0"/>\n' # Anchor point
        xml_body += '    </body>\n'

    # 2. Generate Interconnected Pistons (K14)
    count_piston = 0
    for i in range(len(nodes)):
        for j in range(i + 1, len(nodes)):
            n1 = nodes[i]
            n2 = nodes[j]
            
            p1 = n1['pos']
            p2 = n2['pos']
            
            # Midpoint and Vector
            dx, dy, dz = p2[0]-p1[0], p2[1]-p1[1], p2[2]-p1[2]
            dist = math.sqrt(dx**2 + dy**2 + dz**2)
            if dist < 0.001: continue
            
            # FILTER: ONLY RHOMBIC DODECAHEDRON EDGES (24 Total)
            # 1. Must be different types (Cube <-> Octahedron)
            if n1['type'] == n2['type']: continue
            
            # 2. Must be neighbors (Short edge ~0.866s)
            # s=1.8 -> edge ~ 1.55
            ideal_dist = s * math.sqrt(3) / 2
            if abs(dist - ideal_dist) > (s * 0.1): continue
            
            # Global Midpoint (relative to origin, before z_offset)
            mx, my, mz = (p1[0]+p2[0])/2, (p1[1]+p2[1])/2, (p1[2]+p2[2])/2
            
            p_name = f"edge_{i}_{j}"
            barrel_len = dist * 0.55
            rod_len = dist * 0.55
            th_struct = s * 0.015
            
            # Barrel Body (Floating, at midpoint + z_offset)
            xml_body += f'    <body name="{p_name}_barrel" pos="{mx} {my} {mz + z_offset}" zaxis="{dx} {dy} {dz}">\n'
            xml_body += '        <freejoint/>\n'
            xml_body += f'        <geom type="capsule" fromto="0 0 {-barrel_len/2} 0 0 {barrel_len/2}" size="{th_struct*1.2}" rgba="0 1 1 0.3" mass="{mass_piston}" {d_piston_geom}/>\n'
            
            # Rod Body (Child of Barrel via Slide Joint)
            xml_body += f'        <body name="{p_name}_rod" pos="0 0 0">\n'
            xml_body += f'            <joint name="slide_{p_name}" type="slide" axis="0 0 1" limited="true" range="-0.2 0.2" damping="{joint_damping}" armature="{joint_armature}"/>\n'
            xml_body += f'            <geom type="capsule" fromto="0 0 {-rod_len/2} 0 0 {rod_len/2}" size="{th_struct}" rgba="0 1 1 0.6" mass="{mass_piston}" {d_piston_geom}/>\n'
            xml_body += '        </body>\n'
            xml_body += '    </body>\n'
            
            # Constraints: Connect Barrel/Rod ends to Nodes
            # Barrel Start (Local -dist/2) -> Node 1
            # Rod End (Local +dist/2 relative to Rod Origin) -> Node 2
            # Note: Anchor points need to be GLOBAL coordinates (or relative to world).
            # We calculated p1, p2 local to structure center. 
            # We must add z_offset for global anchor.
            
            ap1 = f"{p1[0]} {p1[1]} {p1[2] + z_offset}"
            ap2 = f"{p2[0]} {p2[1]} {p2[2] + z_offset}"
            
            xml_equality += f'        <connect name="conn_{p_name}_1" body1="{p_name}_barrel" body2="{n1["name"]}" anchor="{ap1}"/>\n'
            xml_equality += f'        <connect name="conn_{p_name}_2" body1="{p_name}_rod" body2="{n2["name"]}" anchor="{ap2}"/>\n'
            
            # Actuator
            xml_actuator += f'        <position name="act_{p_name}" joint="slide_{p_name}" kp="{gain_kp}" kv="{gain_kv}" ctrllimited="true" ctrlrange="-0.5 0.5"/>\n'
            
            # TENSOR (Parallel Tendon)
            t_name = f"tensor_{i}_{j}"
            # Stiffness 1000, Damping 100. Rest length init from qpos0 default.
            xml_tendon += f'    <spatial name="{t_name}" stiffness="1000" damping="100" width="0.005" rgba="1 0.5 0 1">\n'
            xml_tendon += f'        <site site="site_{n1["name"]}"/>\n'
            xml_tendon += f'        <site site="site_{n2["name"]}"/>\n'
            xml_tendon += '    </spatial>\n'
            
            count_piston += 1

    # 3. ROBOT ATTACHMENT (Suspension)
    # We want the robot "Held by the center".
    # Connect Robot Pelvis to the 6 Axis Nodes (Octahedron) via spatial tendons (Springs).
    # This centers the robot elastically.
    
    # Spatial tendons don't work well for "holding" rigid position without oscillation.
    # User said "held". 
    # Let's try Multi-Weld? No, overconstrained.
    # Let's try 6 Distance Constraints (fixed length tendons).
    # Anchor points on robot: Pelvis (0,0,0 local).
    # Anchor points on nodes: Node Centers.
    
    for i in range(6): # First 6 nodes are Octahedron
        n = nodes[i]
        t_name = f"suspend_{i}"
        
        # Spatial Tendon
        # xml_tendon += f'    <spatial name="{t_name}" stiffness="10000" damping="1000" width="0.01" rgba="1 0 0 1">\n'
        # xml_tendon += f'        <site site="site_{n["name"]}"/>\n'
        # xml_tendon += '        <body body="pelvis"/>\n' # Connect to pelvis body origin
        # xml_tendon += '    </spatial>\n'
        
        # Better: Distance Equality Constraint (Rigid Rods to Center)
        # This forces the node to stay at fixed distance from Pelvis? No. 
        # This forces the distance between A and B to be X.
        # If we set it to the initial radius 's', it holds the shape.
        
        # ACTUALLY, simpler: Connect Pelvis to the 6 Axis nodes using 'connect' constraints?
        # A 'connect' constraint fuses a point on body A to a point on body B.
        # If we fuse Pelvis(0,0,0) to Node(0,0,0)... that collapses the node to the pelvis.
        
        # We want the Node to be at (s, 0, 0) relative to Pelvis.
        # So we can define an anchor on the Pelvis at (s, 0, 0).
        # And connect that anchor to the Node.
        # effectively creating invisible struts.
        
        # Anchor on Pelvis (Local Offset)
        px, py, pz = n['pos'] # (e.g. 1.8, 0, 0)
        
        # We can't easily define dynamic anchors in 'connect'.
        # But we can use a 'weld' with 'relpose'.
        # Weld Node_i to Pelvis with relpose = position of node.
        # This makes the 6 Axis nodes RIGIDLY attached to the Pelvis.
        # This matches "Robot held by the center" and gives a solid core.
        # The Cube nodes are then floating around this core, connected by pistons.
        
        xml_equality += f'        <weld name="hold_{i}" body1="pelvis" body2="{n["name"]}" relpose="{px} {py} {pz} 1 0 0 0"/>\n'

    # 4. SURFACE TENDONS - REMOVED (Replaced by Global Tensors in Loop)
    # print(f"Generated {count_tendon} Surface Tensors.")

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
    # Add bodies
    final_xml = final_xml.replace('<worldbody>', f'<worldbody>\n    {floor_xml}\n    {xml_body}')
    
    # Helpers
    def inject_section(tag, content):
        nonlocal final_xml
        if not content: return
        if f'</{tag}>' in final_xml:
            final_xml = final_xml.replace(f'</{tag}>', content + f'\n  </{tag}>')
        else:
            final_xml = final_xml.replace('</mujoco>', f'<{tag}>{content}</{tag}>\n</mujoco>')

    inject_section('actuator', xml_actuator)
    inject_section('tendon', xml_tendon)
    inject_section('equality', xml_equality)

    with open(output_path, "w") as f:
        f.write(final_xml)

    print(f"Generated Simple Scene: {output_path}")

if __name__ == "__main__":
    generate_scene_simple()
