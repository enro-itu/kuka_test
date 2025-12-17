import numpy as np
import csv
import random

# --- 1. KINEMATICS HELPERS ---

def get_rotation_from_rpy(roll, pitch, yaw):
    Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
    Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
    Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx

def get_axis_rotation(axis, theta):
    axis = np.array(axis)
    norm = np.linalg.norm(axis)
    if norm == 0: return np.eye(3)
    axis = axis / norm
    kx, ky, kz = axis
    K = np.array([[0, -kz, ky], [kz, 0, -kx], [-ky, kx, 0]])
    return np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * (K @ K)

def create_transform(xyz, rpy, joint_axis=None, theta=0):
    R_static = get_rotation_from_rpy(rpy[0], rpy[1], rpy[2])
    T = np.eye(4)
    T[:3, :3] = R_static
    T[:3, 3] = np.array(xyz)
    
    if joint_axis is not None:
        R_joint = get_axis_rotation(joint_axis, theta)
        T_joint = np.eye(4)
        T_joint[:3, :3] = R_joint
        return T @ T_joint
    return T

# --- 2. FORWARD KINEMATICS WITH INTERMEDIATE POINTS ---

def get_robot_skeleton(joints):
    """
    Returns the [x,y,z] global coordinates of every joint origin.
    This is used to define the 'bones' of the robot for collision checking.
    """
    q1, q2, q3, q4, q5 = joints

    # 1. Base (Fixed)
    T_01 = create_transform([0,0,0], [0,0,0], [0,0,1], q1)
    
    # 2. Shoulder (Rotary -> Lower)
    T_12 = create_transform([0.06, 0.26, 0.64], [-0.17453, -1.57080, 0], [0,0,1], q2)
    T_02 = T_01 @ T_12 # Global Shoulder Pose

    # 3. Elbow (Lower -> Upper)
    T_23 = create_transform([0.44, 0.48717, -0.05626], [0,0,0], [0,0,1], q3)
    T_03 = T_02 @ T_23 # Global Elbow Pose

    # 4. Wrist Start (Upper -> Support)
    T_34 = create_transform([-0.05, 0.52078, 0.04106], [0.01066, 0.08661, 1.69343], [0.99714, 0, 0.07554], q4)
    T_04 = T_03 @ T_34 # Global Wrist Support Pose

    # 5. Wrist End (Support -> Wrist)
    T_45 = create_transform([0.16404, 0.00001, 0.12645], [1.57080, 1.48353, -0.12217], [0.99714, 0, 0.07554], q5)
    T_05 = T_04 @ T_45 # Global Wrist Pose

    # 6. End Effector
    T_5ee = create_transform([0, 0, 0.05], [0, 0, 0], None, 0)
    T_0ee = T_05 @ T_5ee

    # Extract positions (Translation column)
    points = {
        'base': T_01[:3, 3],
        'shoulder': T_02[:3, 3],
        'elbow': T_03[:3, 3],
        'wrist_start': T_04[:3, 3],
        'wrist_end': T_05[:3, 3],
        'ee': T_0ee[:3, 3]
    }
    return points

# --- 3. GEOMETRY & COLLISION MATH ---

def dist_segment_to_segment(p1, p2, q1, q2):
    """
    Calculates the minimum distance between two line segments (p1-p2) and (q1-q2).
    Used to check if Arm Bone hits Body Bone.
    """
    # This is a simplified check: Distance between two points for speed
    # A full segment-segment dist is expensive.
    # We will use sample points along the arm for collision.
    
    # Generate 5 test points along the arm segment (p1 to p2)
    min_dist = float('inf')
    for t in np.linspace(0, 1, 5):
        point_on_arm = p1 + t * (p2 - p1)
        
        # Check distance to the 'body' segment (q1 to q2)
        # Project point onto line segment
        l2 = np.sum((q2 - q1)**2)
        if l2 == 0: dist = np.linalg.norm(point_on_arm - q1)
        else:
            t_proj = max(0, min(1, np.dot(point_on_arm - q1, q2 - q1) / l2))
            projection = q1 + t_proj * (q2 - q1)
            dist = np.linalg.norm(point_on_arm - projection)
        
        if dist < min_dist:
            min_dist = dist
            
    return min_dist

def is_self_collision(points):
    """
    Returns True if the robot is colliding with itself.
    Define radii (thickness) for each part roughly.
    """
    
    # --- DEFINITIONS ---
    # The 'Column' is the static base tower (Base -> Shoulder)
    col_p1 = np.array([0, 0, 0])
    col_p2 = np.array([0, 0, 0.6]) # Height of rotary joint
    col_radius = 0.15 # 15cm radius for the main tower
    
    # The 'Lower Arm' (Shoulder -> Elbow)
    # Usually doesn't hit the base due to joint limits, but good to check.
    
    # The 'Upper Arm' (Elbow -> Wrist Start)
    upper_p1 = points['elbow']
    upper_p2 = points['wrist_start']
    upper_radius = 0.08 # 8cm radius
    
    # The 'Wrist' (Wrist Start -> EE)
    wrist_p1 = points['wrist_start']
    wrist_p2 = points['ee']
    wrist_radius = 0.05 # 5cm radius
    
    # --- CHECK 1: Wrist hitting the Floor ---
    if points['ee'][2] < 0.05 or points['wrist_end'][2] < 0.05:
        return True # Hit the floor
        
    # --- CHECK 2: Upper Arm hitting the Main Column ---
    dist_upper_col = dist_segment_to_segment(upper_p1, upper_p2, col_p1, col_p2)
    if dist_upper_col < (upper_radius + col_radius):
        return True # Collision!
        
    # --- CHECK 3: Wrist hitting the Main Column ---
    dist_wrist_col = dist_segment_to_segment(wrist_p1, wrist_p2, col_p1, col_p2)
    if dist_wrist_col < (wrist_radius + col_radius):
        return True # Collision!
    
    return False

# --- 4. MAIN GENERATION LOOP ---

def generate_workspace(num_samples=800000, output_file='valid_workspace.csv'):
    
    valid_points = []
    
    print(f"Sampling {num_samples} configurations (this may take a moment)...")
    
    count = 0
    while len(valid_points) < num_samples:
        count += 1
        
        # 1. Random Sample within Limits (radians)
        # Using full URDF limits (-3.14 to 3.14) for all joints
        q1 = random.uniform(-3.14, 3.14) # Base
        q2 = random.uniform(-3.14, 3.14) # Shoulder
        q3 = random.uniform(-3.14, 3.14) # Elbow (Full range)
        q4 = random.uniform(-3.14, 3.14) # Wrist 1
        q5 = random.uniform(-3.14, 3.14) # Wrist 2
        
        joints = [q1, q2, q3, q4, q5]
        
        # 2. Compute Skeleton
        points = get_robot_skeleton(joints)
        
        # 3. Check Collision
        if not is_self_collision(points):
            # 4. Save if Valid
            ee = points['ee']
            # Save ONLY: x, y, z
            valid_points.append([ee[0], ee[1], ee[2]])

    # Write to CSV
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['X', 'Y', 'Z'])
        writer.writerows(valid_points)
        
    print(f"Generated {len(valid_points)} valid points. Saved to {output_file}.")

if __name__ == "__main__":
    generate_workspace(num_samples=800000)