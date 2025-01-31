import mujoco
import mujoco.viewer

# from mocap_move import key_callback_data, get_point, step_trajectory
from mocap_move2 import get_point, step_trajectory
import time


def main():
    # Load the mujoco model basic.xml
    model = mujoco.MjModel.from_xml_path('clothtest2.xml')
    data = mujoco.MjData(model)
    # breakpoint()

    def key_callback(key):
        key_callback_data(key, data)
    executed=False
    with mujoco.viewer.launch_passive(model, data) as viewer:
        # Close the viewer automatically after 30 wall-seconds.
        start = time.time()
        while viewer.is_running() and time.time() - start < 2:
            # breakpoint()
            mujoco.mj_step(model,data) # let cloth settle down
            viewer.sync()
        start = time.time()
        while viewer.is_running() and time.time() - start < 50:
            step_start = time.time()

            # mj_step can be replaced with code that also evaluates
            # a policy and applies a control signal before stepping the physics.
            
            if round(time.time(),2)*10 % 0.5 != 0:
                executed = False
            if round(time.time(),2)*10 % 0.5 == 0 and not executed:
                executed = True
                data.mocap_pos = step_trajectory(model, data)
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
    #         mujoco.mj_step(model, data)
    #         viewer.sync()


if __name__ == '__main__':
    main()
