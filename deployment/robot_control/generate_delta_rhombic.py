import math
import os

def generate_scene_delta():
    # Paths
    g1_path = "public/mujoco/menagerie/unitree_g1/g1.xml"
    output_path = "public/mujoco/menagerie/unitree_g1/scene_delta.xml"

    # Read G1 XML
    try:
        with open(g1_path, "r") as f:
            g1_content = f.read()
    except FileNotFoundError:
        print(f"Error: Could not find {g1_path}")
        return

    # --- GEOMETRY SETTINGS ---
    s = 1.8  # Scale
    z_offset = 1.0 
    
    # Delta Mechanism Parameters
    # Crank: Short active arm
    # Rod: Long passive arm
    crank_len_factor = 0.3 
    rod_len_factor = 0.7     
    
    # Damping/Contact settings
    d_joint = 'damping="2.0" armature="0.1"'
    d_geom = 'contype="0" conaffinity="0" solref="0.005 1" solimp="0.9 0.95 0.001" margin="0.001"'
    th_struct = s * 0.015
    
    xml_nodes = ""
    xml_equality = ""
    xml_actuator = ""
    xml_tendon = ""
    xml_contact = ""

    # Helper: Add Delta Leg (Rotary Motor -> Crank -> Rod -> Target)
    def add_delta_leg(base_idx, target_idx, p_base, p_target, name_suffix):
        nonlocal xml_nodes, xml_equality, xml_actuator
        
        leg_name = f"delta_{name_suffix}"
        
        # Vector from Base (Cube) to Target (Octa)
        dx = p_target[0] - p_base[0]
        dy = p_target[1] - p_base[1]
        dz = p_target[2] - p_base[2]
        dist = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # Hinge Axis Calculation
        # The hinge axis should be perpendicular to the connection vector and "horizontal-ish"
        # to allow the crank to rotate "towards" the target.
        # We calculate it as Cross(Vector, Up).
        ux, uy, uz = dx/dist, dy/dist, dz/dist
        
        # Arbitrary Up vector for cross product.
        # If vector is pure Z, use X as Up.
        ax, ay, az = 0, 0, 1
        if abs(uz) > 0.9: ax, ay, az = 1, 0, 0 
        
        # Hinge Axis = Cross(U, A)
        hx = uy*az - uz*ay
        hy = uz*ax - ux*az
        hz = ux*ay - uy*ax
        # Normalize
        h_len = math.sqrt(hx*hx + hy*hy + hz*hz)
        hx, hy, hz = hx/h_len, hy/h_len, hz/h_len
        
        # Crank Length and Rod Length
        L_crank = dist * crank_len_factor
        L_rod = dist * rod_len_factor
        
        # 1. UPPER ARM (Crank) - Active
        # Rotates around Base Node
        xml_nodes += f'    <body name="{leg_name}_crank" pos="{p_base[0]} {p_base[1]} {p_base[2] + z_offset}">\n'
        # Hinge Joint (Motor) - Rotates -90 to 90 degrees
        xml_nodes += f'        <joint name="joint_{leg_name}" type="hinge" axis="{hx} {hy} {hz}" range="-120 120" limited="true" {d_joint}/>\n'
        # Crank Geom (Visual) pointing roughly towards target initially (though physics will settle it)
        # We draw it along the "Zero Angle" which we define as pure extension? Or perpendicular?
        # Let's draw it "along the rod vector" for now, visualization only.
        # Actually, let's draw it perpendicular to hinge axis.
        # Cross(Hinge, Up)? 
        # Simpler: Just drawn along the initial connection vector scaled down.
        xml_nodes += f'        <geom type="capsule" fromto="0 0 0 {dx*crank_len_factor} {dy*crank_len_factor} {dz*crank_len_factor}" size="{s*0.02}" rgba="0.9 0.2 0.2 1" {d_geom}/>\n'
        
        # END OF CRANK (Site for Rod attachment)
        # Position is L_crank along the geometric vector (approximation for initial pos)
        cx, cy, cz = dx*crank_len_factor, dy*crank_len_factor, dz*crank_len_factor
        
        # 2. LOWER ARM (Rod) - Passive Link
        # Attached to Crank Tip via Ball Joint? 
        # Or separate body connected via Equality.
        # Let's make it a child body of the Crank to avoid equality mess at the elbow?
        # No, a 4-bar linkage or Delta leg usually has specific kinematics.
        # Ideally: 
        #   Body "Rod" has a Ball Joint at "Crank Tip".
        #   Body "Rod" has a Connect Constraint to "Target Node".
        
        xml_nodes += f'        <body name="{leg_name}_rod" pos="{cx} {cy} {cz}">\n'
        # Free joint usually needed if using constraints, but here we can use a ball joint relative to parent (Crank)
        # This models the "Elbow" as a ball joint.
        xml_nodes += f'            <joint name="ball_{leg_name}_elbow" type="ball" limited="false" damping="0.1"/>\n' 
        
        # Rod Geom
        # From Elbow (0,0,0) to Target?
        # We need the vector from Crank Tip to Target.
        # Target Pos relative to Crank Tip:
        # T_global - (Base_global + Crank_vec)
        # This is (dx,dy,dz) - (cx,cy,cz) ... valid for initial setup.
        rx, ry, rz = dx - cx, dy - cy, dz - cz
        
        xml_nodes += f'            <geom type="capsule" fromto="0 0 0 {rx} {ry} {rz}" size="{s*0.01}" rgba="0.2 0.6 1 1" {d_geom}/>\n'
        
        # Site at Rod Tip for connection to Target
        xml_nodes += f'            <site name="site_{leg_name}_tip" pos="{rx} {ry} {rz}"/>\n'
        xml_nodes += '        </body>\n'
        xml_nodes += '    </body>\n'
        
        # 3. CONNECTION (Rod Tip -> Target Node)
        # Use Equality Connect (Ball joint constrained)
        # Connect 'site_leg_tip' to 'node_target'
        # Note: bodies must be dynamic. Active nodes are freejoints.
        xml_equality += f'        <connect name="conn_{leg_name}" body1="{leg_name}_rod" body2="node_{target_idx}" anchor="{rx} {ry} {rz}"/>\n'
        
        # 4. ACTUATOR
        # Controls the Hinge Joint at the Base
        # Gain 500
        xml_actuator += f'        <position name="act_{leg_name}" joint="joint_{leg_name}" kp="500" kv="50" ctrllimited="true" ctrlrange="-2.0 2.0"/>\n'

    # --- 1. OCTAHEDRON NODES (0-5) - Inner ---
    print(f"Generating Octahedron Nodes: {len(axis_verts)}")
    for i, v in enumerate(axis_verts):
        xml_nodes += '    <body name="node_{}" pos="{} {} {}" gravcomp="1">\n'.format(i, v[0], v[1], v[2] + z_offset)
        xml_nodes += '        <freejoint/>\n'
        xml_nodes += '        <site name="node_{}"/>\n'.format(i)
        xml_nodes += '        <geom type="sphere" size="{}" rgba="0.8 0.5 0.2 1" mass="2.0" {}/>\n'.format(s*0.04, d_geom)
        xml_nodes += '    </body>\n'

    # --- 2. CUBE NODES (6-13) - Outer/Base ---
    cube_coords = []
    for x in [-1, 1]:
        for y in [-1, 1]:
            for z in [-1, 1]:
                cube_coords.append((x * 0.5 * s, y * 0.5 * s, z * 0.5 * s))

    print(f"Generating Cube Nodes: {len(cube_coords)}")
    for i, v in enumerate(cube_coords):
        idx = 6 + i
        xml_nodes += '    <body name="node_{}" pos="{} {} {}" gravcomp="1">\n'.format(idx, v[0], v[1], v[2] + z_offset)
        xml_nodes += '        <freejoint/>\n'
        xml_nodes += '        <site name="node_{}"/>\n'.format(idx)
        xml_nodes += '        <geom type="box" size="{}" rgba="0.2 0.8 0.2 1" mass="10.0" {}/>\n'.format(s*0.05, d_geom) # Box for base
        xml_nodes += '    </body>\n'

    # --- 3. DELTA LEGS (Connecting Cube Base to Octa Target) ---
    rd_edge_len = math.sqrt(0.75) * s
    rd_tol = 0.05 * s
    count_delta = 0
    
    print(f"Generating 24 Delta Legs...")
    # Iterate through CUBE nodes (Bases)
    for j in range(len(cube_coords)):
        base_idx = 6 + j # Cube Node Index
        
        # Find adjacent OCTA nodes
        neighbors = []
        for i in range(len(axis_verts)):
             dist = math.sqrt(sum((axis_verts[i][k]-cube_coords[j][k])**2 for k in range(3)))
             if abs(dist - rd_edge_len) < rd_tol:
                 neighbors.append(i)
        
        print(f"Cube Node {base_idx} connects to Octa Nodes: {neighbors}")
        
        # Create a Delta Leg for each neighbor
        for target_idx in neighbors:
             add_delta_leg(base_idx, target_idx, cube_coords[j], axis_verts[target_idx], f"{base_idx}_to_{target_idx}")
             count_delta += 1
             
    # --- 4. TENDONS for Shell Integrity ---
    # We still need tendons to define the shapes (Octahedron and Cube) or they are just loose points.
    
    # Octahedron Tendons
    oct_edge_len = s * math.sqrt(2)
    count_oct = 0
    for i in range(len(axis_verts)):
        for j in range(i + 1, len(axis_verts)):
            dist = math.sqrt(sum((axis_verts[i][k]-axis_verts[j][k])**2 for k in range(3)))
            if abs(dist - oct_edge_len) < 0.05*s:
                t_name = f"tendon_oct_{count_oct}"
                xml_tendon += f'        <spatial name="{t_name}" stiffness="2000" damping="50" width="0.005" rgba="0.8 0.5 0.2 0.5">\n'
                xml_tendon += f'            <site site="node_{i}"/>\n'
                xml_tendon += f'            <site site="node_{j}"/>\n'
                xml_tendon += '        </spatial>\n'
                count_oct += 1
                
    # Cube Tendons
    cube_edge_len = s
    count_cube = 0
    for i in range(len(cube_coords)):
        for j in range(i + 1, len(cube_coords)):
            dist = math.sqrt(sum((cube_coords[i][k]-cube_coords[j][k])**2 for k in range(3)))
            if abs(dist - cube_edge_len) < 0.05*s:
                t_name = f"tendon_cube_{count_cube}"
                xml_tendon += f'        <spatial name="{t_name}" stiffness="2000" damping="50" width="0.005" rgba="0 1 0 0.5">\n'
                xml_tendon += f'            <site site="node_{6+i}"/>\n'
                xml_tendon += f'            <site site="node_{6+j}"/>\n'
                xml_tendon += '        </spatial>\n'
                count_cube += 1

    # --- 5. ROBOT ATTACHMENT ---
    # Weld Robot Pelvis to Center Anchor 
    xml_nodes += '    <body name="center_anchor" pos="0 0 {}">\n'.format(z_offset)
    xml_nodes += '        <site name="center_site"/>\n'
    xml_nodes += '    </body>\n'
    xml_equality += '        <weld name="attach_robot" body1="center_anchor" body2="pelvis" relpose="0 0 0 1 0 0 0"/>\n'
    
    # Suspend Robot from Octahedron Nodes
    for i in range(6):
         xml_tendon += f'        <spatial name="suspension_{i}" stiffness="5000" damping="100" width="0.005" rgba="1 1 1 0.3">\n'
         xml_tendon += f'            <site site="center_site"/>\n'
         xml_tendon += f'            <site site="node_{i}"/>\n'
         xml_tendon += '        </spatial>\n'

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
        
    floor_xml = '<geom name="floor" size="0 0 0.05" type="plane" material="groundplane"/>'
    final_xml = final_xml.replace('<worldbody>', f'<worldbody>\n    {floor_xml}\n    {xml_nodes}')
    
    if '</actuator>' in final_xml:
        final_xml = final_xml.replace('</actuator>', xml_actuator + '\n  </actuator>')
    else:
        final_xml = final_xml.replace('</mujoco>', f'<actuator>{xml_actuator}</actuator>\n</mujoco>')
        
    if xml_tendon:
        if '</tendon>' in final_xml:
             final_xml = final_xml.replace('</tendon>', xml_tendon + '\n  </tendon>')
        else:
             final_xml = final_xml.replace('</mujoco>', f'<tendon>{xml_tendon}</tendon>\n</mujoco>')

    if xml_equality:
        if '</equality>' in final_xml:
             final_xml = final_xml.replace('</equality>', xml_equality + '\n  </equality>')
        else:
             final_xml = final_xml.replace('</mujoco>', f'<equality>{xml_equality}</equality>\n</mujoco>')

    with open(output_path, "w") as f:
        f.write(final_xml)

    print(f"Generated Merged Scene: {output_path}")

if __name__ == "__main__":
    generate_scene_delta()
