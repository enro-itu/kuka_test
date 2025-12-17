import numpy as np

def fk(j1, j2, j3, j4, j5):
    
    # --- 1. Define Link Parameters (copied from URDF) ---
    # This matches the "links = {...}" line in your template
    links = {
        "base":    {"xyz": [0, 0, 0],                  "rpy": [0, 0, 0]},
        "rotary":  {"xyz": [0.06, 0.26, 0.64],         "rpy": [-0.17453, -1.57080, 0.0]},
        "lower":   {"xyz": [0.44, 0.48717, -0.05626],  "rpy": [0, 0, 0]},
        "upper":   {"xyz": [-0.05, 0.52078, 0.04106],  "rpy": [0.01066, 0.08661, 1.69343]},
        "support": {"xyz": [0.16404, 0.00001, 0.12645],"rpy": [1.57080, 1.48353, -0.12217]},
        "end":     {"xyz": [0, 0, 0.05],               "rpy": [0, 0, 0]}
    }

    # --- 2. Helper: Create Matrix from URDF params ---
    def make_tf(link_name, axis, angle):
        # Get constants
        xyz = links[link_name]["xyz"]
        r, p, y = links[link_name]["rpy"]
        
        # 1. Translation Matrix
        Tran = np.eye(4)
        Tran[:3, 3] = xyz
        
        # 2. Static Rotation (Roll-Pitch-Yaw)
        Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
        Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
        Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
        R_static = Rz @ Ry @ Rx

        # 3. Dynamic Joint Rotation (Axis-Angle)
        # Using Rodrigues formula components
        ux, uy, uz = axis
        c, s = np.cos(angle), np.sin(angle)
        K = np.array([[0, -uz, uy], [uz, 0, -ux], [-uy, ux, 0]])
        R_dynamic = np.eye(3) + s*K + (1-c)*(K@K)

        # Combine: Translation * Static Rot * Dynamic Rot
        T = Tran
        T[:3, :3] = R_static @ R_dynamic
        return T

    # --- 3. Compute Chain ---
    # Standard Z-axis for first 3 joints
    T1 = make_tf("base",    [0, 0, 1], j1)
    T2 = make_tf("rotary",  [0, 0, 1], j2)
    T3 = make_tf("lower",   [0, 0, 1], j3)
    
    # Custom Axis for last 2 joints (from URDF)
    custom_axis = [0.99714, 0, 0.07554]
    # Normalize axis
    custom_axis = np.array(custom_axis) / np.linalg.norm(custom_axis)
    
    T4 = make_tf("upper",   custom_axis, j4)
    T5 = make_tf("support", custom_axis, j5)
    
    # End effector (Fixed)
    Te = make_tf("end",     [0, 0, 1], 0)

    # --- 4. Multiply ---
    # T = T1 * T2 * T3 * T4 * T5 * Te
    T_final = T1 @ T2 @ T3 @ T4 @ T5 @ Te

    x, y, z = round(T_final[0, 3],4), round(T_final[1, 3],4), round(T_final[2, 3],4)
    return x, y, z