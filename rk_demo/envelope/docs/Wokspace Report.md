# Wokspace Report

This project aims to identify workspace envelope for robot arm. It includes plot pictures and data samples.

## URDF Joint Limits Table:
joint_limits = {
'base_to_rotary': (-3.14, 3.14),
'rotary_to_lower': (-3.14, 3.14),
'lower_to_upper': (-3.14, 3.14),
'upper_to_support': (-3.14, 3.14),
'support_to_wrist': (-3.14, 3.14),
}

## Parameters (sampling resolution, total samples)

This workspace created by 800000 randomly distributed sample points. Randomly generated entries gave us x,y,z values of the end effector.


![Robot Arm Workspace Plot](..\figures\workspace.png)


## Reaches:

| Axis | Minimum (m) | Maximum (m) | Total Span (m) |
| :--- | :--- | :--- | :--- |
| X | -1.6763 | 1.6734 | 3.3497 |
| Y | -1.6781 | 1.6723 | 3.3504 |
| Z | 0.0500 | 2.0510 | 2.0010 |

Note: This Envelope supposedly prevent robot from doing physically impossible moves. But in return the generator functions and file formats are not like in the work description. Also the robot model that is used has only 5 rotating joints, the envelope is prepared accordingly
