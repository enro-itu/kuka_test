import xml.etree.ElementTree as ET

def get_joint_limits(urdf_path):
    # 1. Load the URDF file
    tree = ET.parse(urdf_path)
    root = tree.getroot()

    # Dictionary to store the results
    joint_limits = {}

    # 2. Loop through every <joint> tag in the file
    for joint in root.findall('joint'):
        joint_name = joint.get('name')
        joint_type = joint.get('type')

        # We only want joints that move (revolute or continuous)
        if joint_type in ['revolute', 'continuous']:
            
            # Find the <limit> tag inside the joint
            limit = joint.find('limit')
            
            if limit is not None:
                # Extract the numbers, convert them to float
                # Default to 0.0 if the attribute is missing
                lower = float(limit.get('lower', 0.0))
                upper = float(limit.get('upper', 0.0))
                
                # Store in the dictionary
                joint_limits[joint_name] = (lower, upper)

    return joint_limits

# --- Run the function ---
if __name__ == "__main__":
    file_path = '/home/arifey/rk_ws/robot_arm.urdf'
    
    
    limits_dict = get_joint_limits(file_path)
    
    # Print the final dictionary
    print("joint_limits = {")
    for name, (low, high) in limits_dict.items():
        print(f"    '{name}': ({low}, {high}),")
    print("}")
