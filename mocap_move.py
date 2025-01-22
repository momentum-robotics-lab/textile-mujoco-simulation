import mujoco
import mujoco.viewer
import numpy as np
import pyquaternion as pyq
import time


def rotate_quaternion(quat, axis, angle):
    """
    Rotate a quaternion by an angle around an axis
    """
    angle_rad = np.deg2rad(angle)
    axis = axis / np.linalg.norm(axis)
    q = pyq.Quaternion(quat)
    q = q * pyq.Quaternion(axis=axis, angle=angle_rad)
    return q.elements


counter = 5
def trajectory(x):
    return 2/x

def trajectory_x(counter):

def get_point(trajectory_function, mocap_pos):
    global counter
    mocap_pos[:,0] += 0.01
    print(trajectory_function(counter))
    if mocap_pos[0,2]>0.01:
        mocap_pos[:,2] -= trajectory_function(counter)*0.05
    else:
        mocap_pos[:,2] = 0.01
    print(mocap_pos)
    return mocap_pos
    
def step_trajectory(model, data):
    global counter, trajectory
    data.mocap_pos = get_point(trajectory, data.mocap_pos)
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

    # if key == 265:  # Up arrow
    #     data.mocap_pos[:, 2] += 0.01
    # elif key == 264:  # Down arrow
    #     data.mocap_pos[:, 2] -= 0.01
    # elif key == 263:  # Left arrow
    #     data.mocap_pos[:, 0] -= 0.01
    # elif key == 262:  # Right arrow
    #     data.mocap_pos[:, 0] += 0.01
    # elif key == 320:  # Numpad 0
    #     data.mocap_pos[:, 1] += 0.01
    # elif key == 330:  # Numpad .
    #     data.mocap_pos[:, 1] -= 0.01
    # elif key == 87:  # Insert
    #     data.mocap_quat[0] = rotate_quaternion(data.mocap_quat[0], [1, 0, 0], 10)
    # elif key == 65:  # Home
    #     data.mocap_quat[0] = rotate_quaternion(data.mocap_quat[0], [1, 0, 0], -10)
    # elif key == 83:  # Home
    #     data.mocap_quat[0] = rotate_quaternion(data.mocap_quat[0], [0, 1, 0], 10)
    # elif key == 68:  # End
    #     data.mocap_quat[0] = rotate_quaternion(data.mocap_quat[0], [0, 1, 0], -10)
    # elif key == 90:  # Page Up
    #     data.mocap_quat[0] = rotate_quaternion(data.mocap_quat[0], [0, 0, 1], 10)
    # elif key == 88:  # Page Down
    #     data.mocap_quat[0] = rotate_quaternion(data.mocap_quat[0], [0, 0, 1], -10)
    # else:
    #     print(key)


def main():
    # Load the mujoco model basic.xml
    model = mujoco.MjModel.from_xml_path('mocap.xml')
    data = mujoco.MjData(model)

    # def key_callback(key):
    #     key_callback_data(key, data)
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Close the viewer automatically after 30 wall-seconds.
        start = time.time()
        while viewer.is_running() and time.time() - start < 50:
            step_start = time.time()

            # mj_step can be replaced with code that also evaluates
            # a policy and applies a control signal before stepping the physics.
            mujoco.mj_step(model, data)

            # Example modification of a viewer option: toggle contact points every two seconds.
            with viewer.lock():
                viewer.opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = int(data.time % 2)

            # Pick up changes to the physics state, apply perturbations, update options from GUI.
            viewer.sync()

            # Rudimentary time keeping, will drift relative to wall clock.
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    # with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
    #     while viewer.is_running():
    #         # modify_mocap(data)
    #         data.mocap_pos = get_point(trajectory, data.mocap_pos)
    #         print(data.mocap_pos)
    #         counter+=1

    #         mujoco.mj_step(model, data)
    #         viewer.sync()


if __name__ == '__main__':
    main()
