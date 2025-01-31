import mujoco
import mujoco.viewer
import numpy as np
import pyquaternion as pyq
import time
from scipy.interpolate import interp1d

# Example keypoints for a simple cloth lift, fling, and set-down motion
# Each array contains (time, value) pairs
L_x_keypoints = np.array([[0, 0], [1, 0.1], [2, 0.4], [4, -0.3], [5, -0.6],[1000,-0.6]])
L_y_keypoints = np.array([[0, 0.175], [10, 0.175], [20, 0.175], [30, 0.175], [40, 0.175],[1000,0.175]])
L_z_keypoints = np.array([[0, 0.3], [1, 0.4], [2, 0.4], [4, 0.2], [5, 0],[1000,0]])
L_rx_keypoints = np.array([[0, 0], [10, 0], [20, 0], [30, 0], [40, 0],[1000,0]])
L_ry_keypoints = np.array([[0, 0], [1, -90], [20, -90], [30, -90], [40, -90],[1000,-90]])
L_rz_keypoints = np.array([[0, 0], [10, 0], [20, 0], [30, 0], [40, 0],[1000,0]])

# Right-hand side keypoints (mirror or customize as needed)
R_x_keypoints = np.array([[0, 0], [1, 0.1], [2, 0.4], [4, -0.3], [5, -0.6],[1000,-0.6]])
R_y_keypoints = np.array([[0, -0.225], [10, -0.225], [20, -0.225], [30, -0.225], [40, -0.225],[1000,-0.225]])
R_z_keypoints = np.array([[0, 0.3], [1, 0.4], [2, 0.4], [4, 0.2], [5, 0],[1000,0]])
R_rx_keypoints = np.array([[0, 0], [10, 0], [20, 0], [30, 0], [40, 0],[1000,0]])
R_ry_keypoints = np.array([[0, 0], [2, -90], [20, -90], [30, -90], [40, -90],[1000,-90]])
R_rz_keypoints = np.array([[0, 0], [10, 0], [20, 0], [30, 0], [40, 0],[1000,0]])

# Create interpolation functions for left and right
L_x_interp = interp1d(L_x_keypoints[:, 0], L_x_keypoints[:, 1], kind='linear', fill_value="extrapolate")
L_y_interp = interp1d(L_y_keypoints[:, 0], L_y_keypoints[:, 1], kind='linear', fill_value="extrapolate")
L_z_interp = interp1d(L_z_keypoints[:, 0], L_z_keypoints[:, 1], kind='linear', fill_value="extrapolate")
L_rx_interp = interp1d(L_rx_keypoints[:, 0], L_rx_keypoints[:, 1], kind='linear', fill_value="extrapolate")
L_ry_interp = interp1d(L_ry_keypoints[:, 0], L_ry_keypoints[:, 1], kind='linear', fill_value="extrapolate")
L_rz_interp = interp1d(L_rz_keypoints[:, 0], L_rz_keypoints[:, 1], kind='linear', fill_value="extrapolate")

R_x_interp = interp1d(R_x_keypoints[:, 0], R_x_keypoints[:, 1], kind='linear', fill_value="extrapolate")
R_y_interp = interp1d(R_y_keypoints[:, 0], R_y_keypoints[:, 1], kind='linear', fill_value="extrapolate")
R_z_interp = interp1d(R_z_keypoints[:, 0], R_z_keypoints[:, 1], kind='linear', fill_value="extrapolate")
R_rx_interp = interp1d(R_rx_keypoints[:, 0], R_rx_keypoints[:, 1], kind='linear', fill_value="extrapolate")
R_ry_interp = interp1d(R_ry_keypoints[:, 0], R_ry_keypoints[:, 1], kind='linear', fill_value="extrapolate")
R_rz_interp = interp1d(R_rz_keypoints[:, 0], R_rz_keypoints[:, 1], kind='linear', fill_value="extrapolate")

counter = 1
last_L_angle, last_R_angle = [0, 0, 0, 0], [0, 0, 0, 0]

def quaternion_to_angle(quat, axis):
    """
    Converts a quaternion rotation to an angle around a specified axis.
    """
    axis = axis / np.linalg.norm(axis)
    q = pyq.Quaternion(quat)
    rot_axis = np.array([q[1], q[2], q[3]])
    angle_rad = 2 * np.arctan2(np.linalg.norm(rot_axis), q[0])
    return np.rad2deg(angle_rad) if np.dot(rot_axis, axis) >= 0 else -np.rad2deg(angle_rad)

# def rotate_quaternion(quat, axis, angle):
#     """
#     Rotate a quaternion by an angle around an axis.
#     """
#     angle_rad = np.deg2rad(angle)
#     axis = axis / np.linalg.norm(axis)
#     q = pyq.Quaternion(quat)
#     q = q * pyq.Quaternion(axis=axis, angle=angle_rad)
#     return q.elements


def rotate_to_quaternion(current_quat, target_angle, axis):
    """
    Rotates a quaternion to a target angle around a specified axis.
    """
    axis = axis / np.linalg.norm(axis)
    current_angle = quaternion_to_angle(current_quat, axis)
    delta_angle = target_angle - current_angle
    delta_angle_rad = np.deg2rad(delta_angle)
    q = pyq.Quaternion(axis=axis, angle=delta_angle_rad)
    return (q * pyq.Quaternion(current_quat)).elements

def L_trajectory_x(t):
    """
    Interpolates the x-axis trajectory for the left hand.
    """
    return L_x_interp(t)

def L_trajectory_y(t):
    """
    Interpolates the y-axis trajectory for the left hand.
    """
    return L_y_interp(t)

def L_trajectory_z(t):
    """
    Interpolates the z-axis trajectory for the left hand.
    """
    return L_z_interp(t)

def L_trajectory_rx(t):
    """
    Interpolates the rotation around the x-axis for the left hand.
    """
    return L_rx_interp(t)

def L_trajectory_ry(t):
    """
    Interpolates the rotation around the y-axis for the left hand.
    """
    return L_ry_interp(t)

def L_trajectory_rz(t):
    """
    Interpolates the rotation around the z-axis for the left hand.
    """
    return L_rz_interp(t)

def R_trajectory_x(t):
    """
    Interpolates the x-axis trajectory for the right hand.
    """
    return R_x_interp(t)

def R_trajectory_y(t):
    """
    Interpolates the y-axis trajectory for the right hand.
    """
    return R_y_interp(t)

def R_trajectory_z(t):
    """
    Interpolates the z-axis trajectory for the right hand.
    """
    return R_z_interp(t)

def R_trajectory_rx(t):
    """
    Interpolates the rotation around the x-axis for the right hand.
    """
    return R_rx_interp(t)

def R_trajectory_ry(t):
    """
    Interpolates the rotation around the y-axis for the right hand.
    """
    return R_ry_interp(t)

def R_trajectory_rz(t):
    """
    Interpolates the rotation around the z-axis for the right hand.
    """
    return R_rz_interp(t)

def get_point(data):
    """
    Updates the mocap position and rotation based on the interpolated trajectory.
    """
    global counter, last_L_angle, last_R_angle

    # Update left hand position and rotation
    data.mocap_pos[0, 0] = L_trajectory_x(counter)
    data.mocap_pos[0, 1] = L_trajectory_y(counter)
    data.mocap_pos[0, 2] = L_trajectory_z(counter)
    target_angle_L = L_trajectory_ry(counter)
    data.mocap_quat[0] = rotate_to_quaternion(data.mocap_quat[0], target_angle_L, [0, 1, 0])
    last_L_angle = data.mocap_quat[0]

    # Update right hand position and rotation
    data.mocap_pos[1, 0] = R_trajectory_x(counter)
    data.mocap_pos[1, 1] = R_trajectory_y(counter)
    data.mocap_pos[1, 2] = R_trajectory_z(counter)
    target_angle_R = R_trajectory_ry(counter)
    data.mocap_quat[1] = rotate_to_quaternion(data.mocap_quat[1], target_angle_R, [0, 1, 0])
    last_R_angle = data.mocap_quat[1]
    print(data.mocap_pos[0])
    return data

def step_trajectory(model, data):
    """
    Steps the simulation and updates the trajectory.
    """
    global counter
    data = get_point(data)
    counter += 0.1  # Increment time
    return data.mocap_pos

# Example of using these functions to simulate the fling:
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    t_values = np.linspace(0, 40, 400)

    L_x_values = [L_trajectory_x(t) for t in t_values]
    L_y_values = [L_trajectory_y(t) for t in t_values]
    L_z_values = [L_trajectory_z(t) for t in t_values]
    L_rx_values = [L_trajectory_rx(t) for t in t_values]

    R_x_values = [R_trajectory_x(t) for t in t_values]
    R_y_values = [R_trajectory_y(t) for t in t_values]
    R_z_values = [R_trajectory_z(t) for t in t_values]
    R_rx_values = [R_trajectory_rx(t) for t in t_values]

    plt.figure(figsize=(10, 8))

    # Plot left hand trajectories
    plt.subplot(3, 1, 1)
    plt.plot(t_values, L_x_values, label='L_x-axis')
    plt.plot(t_values, L_y_values, label='L_y-axis')
    plt.plot(t_values, L_z_values, label='L_z-axis')
    plt.title('Left Hand Position Trajectories')
    plt.legend()

    # Plot right hand trajectories
    plt.subplot(3, 1, 2)
    plt.plot(t_values, R_x_values, label='R_x-axis')
    plt.plot(t_values, R_y_values, label='R_y-axis')
    plt.plot(t_values, R_z_values, label='R_z-axis')
    plt.title('Right Hand Position Trajectories')
    plt.legend()

    # Plot wrist rotation
    plt.subplot(3, 1, 3)
    plt.plot(t_values, L_rx_values, label='L_rx (wrist rotation)')
    plt.plot(t_values, R_rx_values, label='R_rx (wrist rotation)')
    plt.title('Wrist Rotation Trajectory')
    plt.legend()

    plt.tight_layout()
    plt.show()
    