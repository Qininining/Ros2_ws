import numpy as np
from scipy.spatial.transform import Rotation

# The calibration data string provided by the user
kinematics_data_string = """
kinematics:
  shoulder:
    x: 0
    y: 0
    z: 0.089201138580847966
    roll: -0
    pitch: 0
    yaw: 1.5413963267648517e-05
  upper_arm:
    x: 8.28481932000539e-05
    y: 0
    z: 0
    roll: 1.5702793305568989
    pitch: 0
    yaw: 9.4248239648454113e-05
  forearm:
    x: -0.42514603197027351
    y: 0
    z: 0
    roll: 0.00027727163297603633
    pitch: 0.00074181659174685764
    yaw: 8.2061777589927939e-05
  wrist_1:
    x: -0.39231335412023205
    y: 0.00070367842866065884
    z: 0.11004902881674888
    roll: 3.1351985141209435
    pitch: -3.1400954402443415
    yaw: 3.1415185610293745
  wrist_2:
    x: 3.6501405590628345e-05
    y: -0.094721186124807052
    z: 0.0001791428279527357
    roll: 1.5689050644312028
    pitch: 0
    yaw: -1.320607040336194e-05
  wrist_3:
    x: 0.00015399008043081661
    y: 0.082421327089370319
    z: 0.00012632686207930083
    roll: 1.5723290218826846
    pitch: 3.1415926535897931
    yaw: -3.14158677108863
  hash: calib_15200003067234149431
"""

def parse_kinematics_data(data_string):
    """Parses the kinematics data string into a dictionary."""
    parsed_data = {}
    current_block_name = None
    lines = data_string.strip().split('\n')
    
    for line in lines:
        stripped_line = line.strip()
        if not stripped_line or stripped_line.startswith('hash:'):
            continue
        
        if stripped_line.endswith(':'):
            # This is a main block like 'kinematics:' or a component name like 'shoulder:'
            # We are interested in component names under 'kinematics:'
            potential_block_name = stripped_line[:-1]
            # Check if this is a component name by looking at indentation or context
            if line.startswith("  ") and not potential_block_name == "kinematics": # Simple check for component
                 current_block_name = potential_block_name
                 parsed_data[current_block_name] = {}
            elif potential_block_name == "kinematics": # Skip the "kinematics:" line itself
                pass

        elif current_block_name and ':' in stripped_line:
            # This is a key-value pair for the current component
            key, value = stripped_line.split(':', 1)
            parsed_data[current_block_name][key.strip()] = float(value.strip())
            
    return parsed_data

def create_transform_matrix(x, y, z, roll, pitch, yaw):
    """
    Creates a 4x4 homogeneous transformation matrix from x, y, z, roll, pitch, yaw.
    Roll, Pitch, Yaw are intrinsic rotations around X, Y', Z'' respectively.
    """
    # R = R_x(roll) * R_y(pitch) * R_z(yaw) - intrinsic rotations
    rotation = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=False)
    R = rotation.as_matrix()
    
    T = np.eye(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = [x, y, z]
    return T

def main():
    """
    Main function to calculate and print screw axes and M matrix.
    """
    # Parse the provided kinematics data
    # The parser expects component data to be indented.
    # If the top "kinematics:" line is present and not indented, the simple parser might need adjustment
    # For the given string, we manually select the components.
    
    # Manual extraction for this specific format
    data_lines = kinematics_data_string.strip().split('\n')
    components = {}
    current_component = None
    for line in data_lines:
        line_stripped = line.strip()
        if line_stripped.endswith(':') and line.startswith('  ') and ' ' not in line_stripped[:-1]: # e.g. "  shoulder:"
            current_component = line_stripped[:-1]
            components[current_component] = {}
        elif current_component and ':' in line_stripped and line.startswith('    '): # e.g. "    x: 0"
            key, val_str = line_stripped.split(':', 1)
            components[current_component][key.strip()] = float(val_str.strip())

    parsed_data = components

    # Standard local joint axis directions for UR5 (Z, Y, Y, Y, Z, Y)
    local_joint_axes_unit_vectors = [
        np.array([0, 0, 1]),  # Joint 1 (Base Z-axis)
        np.array([0, 1, 0]),  # Joint 2 (Shoulder link Y-axis)
        np.array([0, 1, 0]),  # Joint 3 (UpperArm link Y-axis)
        np.array([0, 1, 0]),  # Joint 4 (Forearm link Y-axis)
        np.array([0, 0, 1]),  # Joint 5 (Wrist1 link Z-axis)
        np.array([0, 1, 0])   # Joint 6 (Wrist2 link Y-axis)
    ]

    screw_axes_S = []

    # --- Screw Axis S1 (Joint 1: Shoulder Pan) ---
    # Assumed to rotate around the base Z-axis, passing through the base origin.
    omega1_B = local_joint_axes_unit_vectors[0]  # In Base frame
    q1_B = np.array([0, 0, 0])                # Origin of Base frame
    v1_B = -np.cross(omega1_B, q1_B)
    S1 = np.hstack((omega1_B, v1_B))
    screw_axes_S.append(S1)

    # --- Relative transformations from the calibration data ---
    # T_B_S: Transformation from Base (B) to Shoulder frame (S)
    T_B_S = create_transform_matrix(**parsed_data['shoulder'])
    # T_S_UA: Transformation from Shoulder (S) to UpperArm frame (UA)
    T_S_UA = create_transform_matrix(**parsed_data['upper_arm'])
    # T_UA_FA: Transformation from UpperArm (UA) to Forearm frame (FA)
    T_UA_FA = create_transform_matrix(**parsed_data['forearm'])
    # T_FA_W1: Transformation from Forearm (FA) to Wrist1 frame (W1)
    T_FA_W1 = create_transform_matrix(**parsed_data['wrist_1'])
    # T_W1_W2: Transformation from Wrist1 (W1) to Wrist2 frame (W2)
    T_W1_W2 = create_transform_matrix(**parsed_data['wrist_2'])
    # T_W2_W3: Transformation from Wrist2 (W2) to Wrist3 frame (W3 - End Effector)
    T_W2_W3 = create_transform_matrix(**parsed_data['wrist_3'])

    # --- Cumulative transformations from Base frame (B) ---
    # T_B_S is already defined

    # --- Screw Axis S2 (Joint 2: Shoulder Lift) ---
    # Joint 2 axis is local Y-axis of Shoulder frame (S)
    # Point q2 on axis is origin of Shoulder frame S, expressed in Base frame B
    R_B_S = T_B_S[0:3, 0:3]
    p_B_S_as_q2 = T_B_S[0:3, 3]
    omega2_B = R_B_S @ local_joint_axes_unit_vectors[1]
    v2_B = -np.cross(omega2_B, p_B_S_as_q2)
    S2 = np.hstack((omega2_B, v2_B))
    screw_axes_S.append(S2)

    # --- Screw Axis S3 (Joint 3: Elbow) ---
    T_B_UA = T_B_S @ T_S_UA
    R_B_UA = T_B_UA[0:3, 0:3]
    p_B_UA_as_q3 = T_B_UA[0:3, 3]
    omega3_B = R_B_UA @ local_joint_axes_unit_vectors[2]
    v3_B = -np.cross(omega3_B, p_B_UA_as_q3)
    S3 = np.hstack((omega3_B, v3_B))
    screw_axes_S.append(S3)

    # --- Screw Axis S4 (Joint 4: Wrist 1) ---
    T_B_FA = T_B_UA @ T_UA_FA
    R_B_FA = T_B_FA[0:3, 0:3]
    p_B_FA_as_q4 = T_B_FA[0:3, 3]
    omega4_B = R_B_FA @ local_joint_axes_unit_vectors[3]
    v4_B = -np.cross(omega4_B, p_B_FA_as_q4)
    S4 = np.hstack((omega4_B, v4_B))
    screw_axes_S.append(S4)

    # --- Screw Axis S5 (Joint 5: Wrist 2) ---
    T_B_W1 = T_B_FA @ T_FA_W1
    R_B_W1 = T_B_W1[0:3, 0:3]
    p_B_W1_as_q5 = T_B_W1[0:3, 3]
    omega5_B = R_B_W1 @ local_joint_axes_unit_vectors[4]
    v5_B = -np.cross(omega5_B, p_B_W1_as_q5)
    S5 = np.hstack((omega5_B, v5_B))
    screw_axes_S.append(S5)

    # --- Screw Axis S6 (Joint 6: Wrist 3) ---
    T_B_W2 = T_B_W1 @ T_W1_W2
    R_B_W2 = T_B_W2[0:3, 0:3]
    p_B_W2_as_q6 = T_B_W2[0:3, 3]
    omega6_B = R_B_W2 @ local_joint_axes_unit_vectors[5]
    v6_B = -np.cross(omega6_B, p_B_W2_as_q6)
    S6 = np.hstack((omega6_B, v6_B))
    screw_axes_S.append(S6)

    # --- End-Effector Home Configuration M ---
    # M is the pose of Wrist3 frame (W3, tool flange) relative to Base (B) at zero configuration
    M_home = T_B_W2 @ T_W2_W3

    # --- Output the results ---
    print("Screw Axes (S_i = [omega_x, omega_y, omega_z, v_x, v_y, v_z]):")
    for i, S_i in enumerate(screw_axes_S):
        # Format to a consistent number of decimal places for readability
        formatted_S_i = [f"{x:.8f}" for x in S_i]
        print(f"S{i+1}: {formatted_S_i}")

    print("\nEnd-effector home configuration M (4x4 matrix):")
    # Format matrix for readability
    for row in M_home:
        formatted_row = [f"{x:.8f}" for x in row]
        print(f"  {formatted_row}")

if __name__ == '__main__':
    main()


## Example output:
#     Screw Axes (S_i = [omega_x, omega_y, omega_z, v_x, v_y, v_z]):
# S1: ['0.00000000', '0.00000000', '1.00000000', '0.00000000', '0.00000000', '0.00000000']
# S2: ['-0.00001541', '1.00000000', '0.00000000', '-0.08920114', '-0.00000137', '0.00000000']
# S3: ['-0.00000006', '0.00051700', '0.99999987', '-0.00004612', '-0.00008285', '0.00000004']
# S4: ['-0.00008188', '0.00023972', '0.99999997', '-0.00006800', '0.42505586', '-0.00010190']
# S5: ['-0.00064575', '-0.99997779', '0.00663386', '0.08916459', '0.00536369', '0.81719328']
# S6: ['-0.00064573', '-0.99996345', '0.00852507', '-0.00576688', '0.00697019', '0.81714431']

# End-effector home configuration M (4x4 matrix):
#   ['0.99999979', '0.00000977', '-0.00065163', '-0.81714524']
#   ['-0.00065168', '0.00699242', '-0.99997534', '-0.19365652']
#   ['-0.00000521', '0.99997555', '0.00699242', '-0.00424243']
#   ['0.00000000', '0.00000000', '0.00000000', '1.00000000']