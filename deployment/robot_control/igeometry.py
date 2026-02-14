import numpy as np
import struct

def get_rhombic_dodecahedron_verts():
    # Radius/Scale
    r = 0.1
    
    # 14 Vertices
    # 6 "Points" (tips of the axis) - Octahedron-like
    # (±1, 0, 0), (0, ±1, 0), (0, 0, ±1)
    points_axis = [
        [0, 0, 1.0], [0, 0, -1.0],
        [0, 1.0, 0], [0, -1.0, 0],
        [1.0, 0, 0], [-1.0, 0, 0]
    ]
    
    # 8 "Corners" (Cube corners)
    # (±0.5, ±0.5, ±0.5)
    points_corners = []
    for x in [-0.5, 0.5]:
        for y in [-0.5, 0.5]:
            for z in [-0.5, 0.5]:
                points_corners.append([x, y, z])
                
    verts = np.array(points_axis + points_corners) * (r * 2.0) # Scale it
    return verts

def generate_stl(filename="rhrombic_dodecahedron.stl"):
    # Vertices
    # Axis indices: 0:+Z, 1:-Z, 2:+Y, 3:-Y, 4:+X, 5:-X
    # Corners: 6..13
    
    # Faces: 12 Rhombuses. Each Rhombus = 2 Triangles.
    # Total 24 Triangles.
    
    # A rhombus is formed by 2 Axis points and 2 Corner points.
    
    # Reference for vertex indices from `get_rhombic_dodecahedron_verts`:
    # 0: (0,0,1)
    # 2: (0,1,0)
    # 4: (1,0,0)
    # Corner (0.5, 0.5, 0.5) -> Index that matches [0.5, 0.5, 0.5]
    
    # Let's map strict logic:
    # A face exists between every adjacent pair of 'Axis' vertices? No.
    # A face connects: Axis_North, Axis_East, Corner_NE, Axis_North...
    # It's easier: 12 faces.
    # Each face corresponds to an edge of the inner Cube?
    
    # Let's use hull/convex hull approach or hardcode.
    # Hardcoding the 12 rhombi:
    # Top (+Z, index 0) connects to 4 corners: 
    # (+.5,+.5,+.5), (-.5,+.5,+.5), (-.5,-.5,+.5), (+.5,-.5,+.5)
    # And 4 Equator Axis points: (+Y, -Y, +X, -X) i.e. 2,3,4,5
    
    # Face 1 (Top-Front-Right?): 
    # Center of face is typically ... 
    
    # Convex Hull is safest and easiest for "Data".
    from scipy.spatial import ConvexHull
    
    verts = get_rhombic_dodecahedron_verts()
    hull = ConvexHull(verts)
    
    # Write STL
    with open(filename, 'wb') as f:
        # Header (80 bytes)
        f.write(b'\0' * 80)
        # Number of triangles (4 bytes uint32)
        # hull.simplices is (N, 3) array of indices
        n_triangles = len(hull.simplices)
        f.write(struct.pack('<I', n_triangles))
        
        for sim in hull.simplices:
            # Normal (dummy)
            f.write(struct.pack('<3f', 0.0, 0.0, 0.0))
            # Vertices
            for v_idx in sim:
                v = verts[v_idx]
                f.write(struct.pack('<3f', v[0], v[1], v[2]))
            # Attribute byte count
            f.write(struct.pack('<H', 0))

if __name__ == "__main__":
    generate_stl("rhrombic_dodecahedron.stl")
    print("Generated rhrombic_dodecahedron.stl")
