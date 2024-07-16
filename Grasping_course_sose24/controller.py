import numpy as np
import mujoco
from sensor import read_sensor_data, smooth_data

    # Implement PID control for each joint
def pid_control(model, data, joint_ids, actuator_ids, position_goals, integral_errors, kp=1, ki=0.01, kd=0.1):
    for joint_name, target in position_goals.items():
        joint_id = joint_ids[joint_name]
        q = data.qpos[joint_id]
        qdot = data.qvel[joint_id]
        q_error = target - q
        qdot_error = -qdot
        integral_errors[joint_name] += q_error
        integral = integral_errors[joint_name]
        control_input = (kp * q_error) + (ki * integral) + (kd * qdot_error) * model.opt.timestep
        actuator_id = actuator_ids[joint_name.replace('j', 'a')]
        data.ctrl[actuator_id] = control_input
 # Implement impedance control based on contact information and torque sensors
def impedance_control(model, data, joint_ids, contact_info, torque_sensors, sensor_data_buffer, window_size, ctrlrange, kp=1, kd=0.1, ki=0.01):
    torque_data = {key: [] for key in torque_sensors.keys()}

    for info in contact_info:
        contact_pos = info['contact_position']
        finger_name = info['geom1'] if info['geom1'] != 'object' else info['geom2']
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, finger_name)
        jacp = np.zeros((3, model.nv))
        jacr = np.zeros((3, model.nv))
        mujoco.mj_jacBody(model, data, jacp, jacr, body_id)

       # Get current velocity and acceleration
        vel = np.zeros(6)
        acc = np.zeros(6)
        mujoco.mj_objectVelocity(model, data, mujoco.mjtObj.mjOBJ_BODY, body_id, vel, 0)
        mujoco.mj_objectAcceleration(model, data, mujoco.mjtObj.mjOBJ_BODY, body_id, acc, 0)
        
        # Calculate desired torque
        target_pos = np.array([0, 0.05, 0])  # Assume target position as a reference point
        pos_error = target_pos - contact_pos
        force = kp * pos_error - kd * vel[:3] - ki * acc[:3]  # Consider position, velocity, and acceleration errors
        torque = jacp.T @ force

        for sensor_key in torque_sensors:
            real_torque = read_sensor_data(data, torque_sensors[sensor_key])
            if real_torque is not None:
                real_torque = smooth_data(real_torque, sensor_key, sensor_data_buffer, window_size)
                torque_data[sensor_key].append(real_torque)
                print(f"{sensor_key} Real Torque: {real_torque}")

                torque -= real_torque

        for i in range(len(torque)):
            actuator_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, f"{finger_name.replace('distal', 'a').replace('medial', 'a')}{i}")
            if actuator_id != -1:
                torque[i] = np.clip(torque[i], ctrlrange[actuator_id][0], ctrlrange[actuator_id][1])
                data.qfrc_applied[actuator_id] = torque[i]
                print(f"Applied torque to {finger_name} actuator {actuator_id}: {torque[i]}")

    return torque_data

    # Control the lifting of the object to a specified z position
def lift_object(model, data, ctrlrange, z_goal=2.0, kp=1, kd=0.1):
    joint_name = "hand_z_joint"
    joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
    q = data.qpos[joint_id]
    qdot = data.qvel[joint_id]
    q_error = z_goal - q
    qdot_error = -qdot

    control_input = (kp * q_error) + (kd * qdot_error) * model.opt.timestep
    control_input = np.clip(control_input, ctrlrange[joint_id][0], ctrlrange[joint_id][1])
    data.ctrl[joint_id] = control_input
    print(f"Lifting {joint_name}: Pos {q:.3f}, Vel {qdot:.3f}, Target {z_goal}, Ctrl Input {control_input:.3f}")
