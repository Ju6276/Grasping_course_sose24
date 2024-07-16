import os
import numpy as np
import mujoco
import mujoco.viewer
import matplotlib.pyplot as plt
from controller import pid_control, impedance_control, lift_object
from sensor import get_sensor_id, read_sensor_data, smooth_data, check_contact
from utils import get_body_name, print_all_bodies, get_joint_ids, get_actuator_ids

def main():
    # Load the MuJoCo model from an XML file and initialize the viewer
    xml_path = os.path.join(os.path.dirname(__file__), 'wonik_allegro', 'scene_MediumWrap.xml')
    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)
    viewer = mujoco.viewer.launch_passive(model, data)

    # Define target positions for the joints and get their IDs
    joint_goals = {
        "ffj0": 0.4, "ffj1": 1.1, "ffj2": 0.6, "ffj3": 0.6,
        "mfj0": 0.1, "mfj1": 1.1, "mfj2": 0.6, "mfj3": 0.6,
        "rfj0": 0.1, "rfj1": 1.1, "rfj2": 0.6, "rfj3": 0.6,
        "thj0": 1.3, "thj1": 0.2, "thj2": 0.45, "thj3": 0.5
    }
    joint_ids = get_joint_ids(model, joint_goals)
    actuator_ids = get_actuator_ids(model, joint_goals)

# Get the control range of the actuators.
    ctrlrange = model.actuator_ctrlrange
    integral_errors = {key: 0 for key in joint_goals}

    # Initialize torque sensors and data buffers
    torque_sensors = {
        'ffa0': get_sensor_id(model, 'torque_sensor_ffa0'),
        'ffa1': get_sensor_id(model, 'torque_sensor_ffa1'),
        'ffa2': get_sensor_id(model, 'torque_sensor_ffa2'),
        'ffa3': get_sensor_id(model, 'torque_sensor_ffa3'),
        'mfa0': get_sensor_id(model, 'torque_sensor_mfa0'),
        'mfa1': get_sensor_id(model, 'torque_sensor_mfa1'),
        'mfa2': get_sensor_id(model, 'torque_sensor_mfa2'),
        'mfa3': get_sensor_id(model, 'torque_sensor_mfa3'),
        'rfa0': get_sensor_id(model, 'torque_sensor_rfa0'),
        'rfa1': get_sensor_id(model, 'torque_sensor_rfa1'),
        'rfa2': get_sensor_id(model, 'torque_sensor_rfa2'),
        'rfa3': get_sensor_id(model, 'torque_sensor_rfa3'),
        'tha0': get_sensor_id(model, 'torque_sensor_tha0'),
        'tha1': get_sensor_id(model, 'torque_sensor_tha1'),
        'tha2': get_sensor_id(model, 'torque_sensor_tha2'),
        'tha3': get_sensor_id(model, 'torque_sensor_tha3')
    }

    window_size = 10
    sensor_data_buffer = {key: [] for key in torque_sensors.keys()}
    torque_data_all = {key: [] for key in torque_sensors.keys()}

    while True:
        pid_control(model, data, joint_ids, actuator_ids, joint_goals, integral_errors)  # Apply PID control
        mujoco.mj_step(model, data)
        viewer.sync()

        contact_info = check_contact(model, data, "object")
        if len(contact_info) >= 4: # If grasp is successful
            print("Grasp successful! Contact info:")
            for info in contact_info:
                print(f"Geom1: {info['geom1']}, Geom2: {info['geom2']}, Contact position: {info['contact_position']}")

            while True:
                torque_data = impedance_control(model, data, joint_ids, contact_info, torque_sensors, sensor_data_buffer, window_size, ctrlrange) # Apply impedance control
                mujoco.mj_step(model, data)
                viewer.sync()

                for key in torque_data:
                    torque_data_all[key].extend(torque_data[key])

                lift_object(model, data, ctrlrange) # Lift the object

                hand_z_pos = data.qpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "hand_z_joint")]
                if hand_z_pos >= 0.2:  # Check if the object is lifted to the target position
                    print("Object lifted to target z position.")
                    break

            break
    # Plot and save the torque data
    plt.figure(figsize=(12, 8))
    for key, data in torque_data_all.items():
        plt.plot(data, label=key)
    plt.xlabel('Time Steps')
    plt.ylabel('Torque')
    plt.title('Real-time Torque Data During Grasping with Impedance Control')
    plt.legend()
    plt.savefig('torque_data(Impedance Control).png')
    plt.show()

    print("End of simulation.")

if __name__ == "__main__":
    main()

