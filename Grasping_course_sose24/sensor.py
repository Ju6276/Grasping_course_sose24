import numpy as np
import mujoco
from utils import get_body_name, print_all_bodies, get_joint_ids, get_actuator_ids

    # Get the sensor ID by name, return -1 if not found
def get_sensor_id(model, sensor_name):
    try:
        sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
    except ValueError:
        print(f"Sensor {sensor_name} not found in model.")
        return -1
    return sensor_id

    # Read and return sensor data for a given sensor ID
def read_sensor_data(data, sensor_id):
    if sensor_id == -1:
        return None
    return data.sensordata[sensor_id]

    # Smooth sensor data using a moving average
def smooth_data(sensor_data, key, sensor_data_buffer, window_size):
    sensor_data_buffer[key].append(sensor_data)
    if len(sensor_data_buffer[key]) > window_size:
        sensor_data_buffer[key].pop(0)
    return np.mean(sensor_data_buffer[key])

    # Check for contact between specified palm joints and the object
def check_contact(model, data, object_body_name="object", palm_joint_names=None):
    if palm_joint_names is None:
        palm_joint_names = ["ff_distal", "mf_distal", "rf_distal", "th_distal",
                            "ff_medial", "mf_medial", "rf_medial", "th_medial"]
    contact_info = []
    for i in range(data.ncon):
        contact = data.contact[i]
        body1_name = get_body_name(model, model.geom_bodyid[contact.geom1])
        body2_name = get_body_name(model, model.geom_bodyid[contact.geom2])
        if (body1_name in palm_joint_names and body2_name == object_body_name) or \
           (body2_name in palm_joint_names and body1_name == object_body_name):
            contact_info.append({
                'contact_position': contact.pos.copy(),
                'geom1': body1_name,
                'geom2': body2_name
            })
    return contact_info
