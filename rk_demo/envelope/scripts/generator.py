from rk_demo.scripts.parse_joint_limits import joint_limits
import numpy as np
import csv
from fk_func import fk
# print(joint_limits['base_to_rotary'][0])
# print(joint_limits['base_to_rotary'][1])

def d2r(deg):
    rad= np.deg2rad(deg)
    return rad

header=['X','Y','Z']



with open('workspace_points6.csv', 'w', newline='') as file:
    csv_writer=csv.writer(file)
    csv_writer.writerow(header)
    seen_points = set()


    for j1 in np.arange(joint_limits['base_to_rotary'][0], joint_limits['base_to_rotary'][1], d2r(9)):
        for j2 in np.arange(joint_limits['rotary_to_lower'][0], joint_limits['rotary_to_lower'][1], d2r(9)):
            for j3 in np.arange(joint_limits['lower_to_upper'][0], joint_limits['lower_to_upper'][1], d2r(18)):
                for j4 in np.arange(joint_limits['upper_to_support'][0], joint_limits['upper_to_support'][1], d2r(18)):
                    for j5 in np.arange(joint_limits['support_to_wrist'][0], joint_limits['support_to_wrist'][1], d2r(18)):
                        x, y, z = fk(j1, j2, j3, j4, j5)
                        point_key = (round(x, 2), round(y, 2), round(z, 2))
                        print(x, y, z)
                        if point_key not in seen_points:
                            seen_points.add(point_key)
                            csv_writer.writerow([round(x,2), round(y,2), round(z,2)])                        
                        



print("Writing points is completed")
