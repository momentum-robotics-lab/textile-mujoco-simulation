import mujoco
import mujoco.viewer
import numpy as np
import pyquaternion as pyq
import time

counter = 1
last_L_angle, last_R_angle = [0,0,0,0], [0,0,0,0]


def quaternion_to_angle(quat, axis):
    """
    Converts a quaternion rotation to an angle around a specified axis.

    Parameters:
    quat: Quaternion representing the rotation.
    axis: The axis of rotation (e.g., [1, 0, 0] for x-axis).

    Returns:
    Angle of rotation (in degrees) around the specified axis.
    """
    axis = axis / np.linalg.norm(axis)
    q = pyq.Quaternion(quat)
    rot_axis = np.array([q[1], q[2], q[3]])
    angle_rad = 2 * np.arctan2(np.linalg.norm(rot_axis), q[0])
    return np.rad2deg(angle_rad) if np.dot(rot_axis, axis) >= 0 else -np.rad2deg(angle_rad)

def rotate_quaternion(quat, axis, angle):
    """
    Rotate a quaternion by an angle around an axis
    """
    angle_rad = np.deg2rad(angle)
    axis = axis / np.linalg.norm(axis)
    q = pyq.Quaternion(quat)
    q = q * pyq.Quaternion(axis=axis, angle=angle_rad)
    return q.elements


def L_trajectory_x(t, x_coord):
    """
    Simulates the x-axis trajectory of a fling.
    Moves the manipulator to the right in an arc.
    t: Time variable (0 to 1).
    """
    t = -int(t)/10
    print(np.pi * t)
    if (np.pi * t) < -2*np.pi:
        return 0
    return 0.1*np.sin(np.pi * t)

def L_trajectory_y(t, y_coord):
    """
    Simulates the y-axis trajectory of a fling.
    Moves the manipulator upward initially and then forward.
    t: Time variable (0 to 1).
    """
    return 0 # Keeps a constant height

def L_trajectory_z(t, z_coord):
    """
    Simulates the z-axis trajectory of a fling.
    Remains constant during the motion.
    t: Time variable (0 to 1).
    """
    if z_coord <= 0:
        return 0
    return  -0.03 #0.1*(np.cos(np.pi * t) - 1)  # Moves upward and arcs downward

def L_trajectory_rx(t):
    """
    Simulates the rotation around the x-axis for the fling.
    Flips the wrist backward and then forward.
    t: Time variable (0 to 1).
    """
    return 0  # No rotation on x-axis

def L_trajectory_ry(t, last_angle):
    """
    Simulates the rotation around the y-axis for the fling.
    Rotates gradually from 45 degrees to 90 degrees following a sine wave.
    t: Time variable (0 to 1).
    """
    start_angle = np.pi/4
    end_angle = np.pi/2
    # print("L", last_angle)
    print(quaternion_to_angle(last_angle, [0, 1, 0]))
    if quaternion_to_angle(last_angle, [0, 1, 0]) >= 90:
        print("HERE")
        return 0
    return -2*np.pi * np.sin(np.pi * t)


def L_trajectory_rz(t):
    """
    Simulates the rotation around the z-axis for the fling.
    Remains constant during the motion.
    t: Time variable (0 to 1).
    """
    return 0  # No rotation on z-axis


def get_point(data):
    """
    Updates the mocap position and rotation based on the provided trajectory function.

    Parameters:
    trajectory_function: A function that calculates the trajectory value based on time.
    data: An object containing mocap position and quaternion data.

    Returns:
    Updated data object.
    """
    global counter, last_L_angle, last_R_angle

    # data.mocap_pos[:, 0] += L_trajectory_x(counter, data.mocap_pos[0,0]) * 0.1
    # data.mocap_pos[:, 1] += L_trajectory_y(counter, data.mocap_pos[0,1]) * 0.1
    # data.mocap_pos[:, 2] += L_trajectory_z(counter, data.mocap_pos[0,2]) * 0.1

    # Rotate the quaternion for the wrist motion
    angle = L_trajectory_ry(counter, last_L_angle)
    data.mocap_quat[0] = rotate_quaternion(data.mocap_quat[0], [0, 1, 0], angle)*2
    data.mocap_quat[1] = rotate_quaternion(data.mocap_quat[1], [0, 1, 0], angle)*2
    last_L_angle = data.mocap_quat[0]

    # print("Updated mocap position:", data.mocap_pos)
    # print("Updated mocap quaternion:", data.mocap_quat)

    return data

    
def step_trajectory(model, data):
    global counter
    data = get_point(data)
    # print(data.mocap_pos, data.mocap_quat)
    counter+=0.2
    return data.mocap_pos

def key_callback_data(key, data):
    """
    Callback for key presses but with data passed in
    :param key: Key pressed
    :param data:  MjData object
    :return: None
    """
    global counter, trajectory
    print(data.mocap_pos)
    data.mocap_pos = get_point(trajectory, data.mocap_pos)
    print(data.mocap_pos)
    counter+=1
    
    
# Example of using these functions to simulate the fling:
if __name__ == "__main__":
    import matplotlib.pyplot as plt

    t_values = np.linspace(0, 1, 100)

    x_values = [L_trajectory_x(t) for t in t_values]
    y_values = [L_trajectory_y(t) for t in t_values]
    z_values = [L_trajectory_z(t) for t in t_values]
    rx_values = [L_trajectory_rx(t) for t in t_values]

    plt.figure(figsize=(10, 8))

    # Plot x, y, z trajectories
    plt.subplot(3, 1, 1)
    plt.plot(t_values, x_values, label='x-axis')
    plt.plot(t_values, y_values, label='y-axis')
    plt.plot(t_values, z_values, label='z-axis')
    plt.title('Position Trajectories')
    plt.legend()

    # Plot wrist rotation
    plt.subplot(3, 1, 2)
    plt.plot(t_values, rx_values, label='rx (wrist rotation)')
    plt.title('Rotation Trajectory')
    plt.legend()

    plt.tight_layout()
    plt.show()

