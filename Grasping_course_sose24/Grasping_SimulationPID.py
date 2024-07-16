import numpy as np
import mujoco
import os
import glfw
import time
import imageio
import matplotlib.pyplot as plt

# Load the MuJoCo model
xml_path = os.path.join(os.path.dirname(__file__), 'wonik_allegro', 'scene_MediumWrap.xml')
model = mujoco.MjModel.from_xml_path(xml_path)
data = mujoco.MjData(model)

# Initialize GLFW and Create a window for the simulation
if not glfw.init():
    raise Exception("GLFW can't be initialized")

window = glfw.create_window(1200, 900, "Grasping Simulation with PID Controller", None, None)
if not window:
    glfw.terminate()
    raise Exception("GLFW window can't be created")

glfw.make_context_current(window)

# Initialize video recorder
video_path = "grasp_simulation(PID_Controller).mp4"
video_writer = imageio.get_writer(video_path, fps=20)

# Set target positions for joints
joint_goals = {
    "ffj0": 0.4, "ffj1": 1.1, "ffj2": 0.6, "ffj3": 0.6,
    "mfj0": 0.1, "mfj1": 1.1, "mfj2": 0.6, "mfj3": 0.6,
    "rfj0": 0.1, "rfj1": 1.1, "rfj2": 0.6, "rfj3": 0.6,
    "thj0": 1.3, "thj1": 0.2, "thj2": 0.45, "thj3": 0.5
}

# Get joint and actuator IDs
joint_ids = {name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in joint_goals.keys()}
actuator_ids = {name.replace('j', 'a'): mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name.replace('j', 'a')) for name in joint_goals.keys()}

# Initialize integral errors for PID control
integral_errors = {key: 0 for key in joint_goals}

    # Perform PID control for each joint
def pid_control(model, data, joint_ids, actuator_ids, position_goals, integral_errors, kp=0.5, ki=0.01, kd=0.1):
    for joint_name, target in position_goals.items():
        joint_id = joint_ids[joint_name]
        q = data.qpos[joint_id]
        qdot = data.qvel[joint_id]
        q_error = target - q
        qdot_error = -qdot
        integral_errors[joint_name] += q_error  # Accumulate the integral of the error
        integral = integral_errors[joint_name]

        control_input = (kp * q_error) + (ki * integral) + (kd * qdot_error) * model.opt.timestep
        actuator_id = actuator_ids[joint_name.replace('j', 'a')]
        data.ctrl[actuator_id] = control_input
        
        print(f"Joint {joint_name}: Pos {q:.3f}, Vel {qdot:.3f}, Target {target}, Ctrl Input {control_input:.3f}")

# Function to get sensor ID
def get_sensor_id(model, sensor_name):
    try:
        sensor_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, sensor_name)
    except ValueError:
        print(f"Sensor {sensor_name} not found in model.")
        return -1
    return sensor_id

# Function to read sensor data
def read_sensor_data(data, sensor_id):
    if sensor_id == -1:
        return None
    return data.sensordata[sensor_id]

    # Get the name of a body by its ID
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
torque_data_all = {key: [] for key in torque_sensors.keys()}

hand_body_names = [
    "ff_distal", "mf_distal", 
    "rf_distal", "th_distal"
]
    # Get the position of a body by its name
def get_body_name(model, body_id):
    name_start = model.name_bodyadr[body_id]
    name_end = model.names.find(b'\0', name_start)
    body_name = model.names[name_start:name_end].decode('utf-8')
    return body_name

    # Get the position of a body by its name
def get_body_position(model, data, body_name):
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return data.xpos[body_id]

window_size = 10
sensor_data_buffer = {key: [] for key in torque_sensors.keys()}

    # Smooth sensor data using a moving average
def smooth_data(sensor_data, key):
    sensor_data_buffer[key].append(sensor_data)
    if len(sensor_data_buffer[key]) > window_size:
        sensor_data_buffer[key].pop(0)
    return np.mean(sensor_data_buffer[key])

   # Check for contact between hand bodies and the object
def check_contact(model, data, object_body_name="object", hand_body_names=[]):
    contact_info = {name: None for name in hand_body_names}  
    for i in range(data.ncon):
        contact = data.contact[i]
        body1_name = get_body_name(model, model.geom_bodyid[contact.geom1])
        body2_name = get_body_name(model, model.geom_bodyid[contact.geom2])
        if (body1_name in hand_body_names and body2_name == object_body_name):
            contact_info[body1_name] = contact.pos.copy()
            print(f"Contact detected between {body1_name} and {body2_name}")
        elif (body2_name in hand_body_names and body1_name == object_body_name):
            contact_info[body2_name] = contact.pos.copy()
            print(f"Contact detected between {body2_name} and {body1_name}")
    all_fingers_contacted = all(contact_info[name] is not None for name in hand_body_names)
    return all_fingers_contacted, contact_info

# Initialize MuJoCo rendering context and scene
cam = mujoco.MjvCamera()
opt = mujoco.MjvOption()
scn = mujoco.MjvScene(model, maxgeom=1000)
con = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150)

# Set initial camera position to view the front of the hand
cam.azimuth = 180  
cam.elevation = -10  
cam.distance = 2.0  
cam.lookat = np.array([0.0, 0.0, 0.0]) 

# Enable contact visualization
opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTPOINT] = True
opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTFORCE] = True
opt.flags[mujoco.mjtVisFlag.mjVIS_CONTACTSPLIT] = True

# Keyboard callback function for switching viewpoints
def keyboard(window, key, scancode, action, mods):
    if action != glfw.PRESS:
        return
    if key == glfw.KEY_1:
        cam.azimuth = 180  # Front view
        cam.elevation = -10
        cam.distance = 2.0
        cam.lookat = np.array([0.0, 0.0, 0.0])
    elif key == glfw.KEY_2:
        cam.azimuth = -90  # Left view
        cam.elevation = -10
        cam.distance = 2.0
        cam.lookat = np.array([0.0, 0.0, 0.0])
    elif key == glfw.KEY_3:
        cam.azimuth = 0  # Top view
        cam.elevation = -90
        cam.distance = 2.0
        cam.lookat = np.array([0.0, 0.0, 0.0])

glfw.set_key_callback(window, keyboard)

#Main cycle for PID control and window update
grasp_successful = False
hand_z_joint_goal = 0.2  
hand_z_joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "hand_z_joint")
integral_error_hand_z = 0

#Gripping towards the z-axis after successful contact.
def move_hand_z_joint(target_pos, integral_error_hand_z, kp=1, ki=0.1, kd=0.5):
    q = data.qpos[hand_z_joint_id]
    qdot = data.qvel[hand_z_joint_id]
    q_error = target_pos - q
    qdot_error = -qdot
    integral_error_hand_z += q_error  

    control_input = (kp * q_error) + (ki * integral_error_hand_z) + (kd * qdot_error) * model.opt.timestep
    data.ctrl[hand_z_joint_id] = control_input

    print(f"Hand Z Joint: Pos {q:.3f}, Vel {qdot:.3f}, Target {target_pos}, Ctrl Input {control_input:.3f}")

    return integral_error_hand_z, abs(q_error) < 1e-3 and abs(qdot_error) < 1e-3

move_to_target = True
while not glfw.window_should_close(window):
    start_time = time.time()
    
    if not grasp_successful:
        pid_control(model, data, joint_ids, actuator_ids, joint_goals, integral_errors)
    else:
        if move_to_target:
            integral_error_hand_z, reached = move_hand_z_joint(hand_z_joint_goal, integral_error_hand_z)
            if reached:
                print("Hand Z Joint moved to the target position.")
                move_to_target = True
                break

    mujoco.mj_step(model, data)

    for sensor_key in torque_sensors:
        real_torque = read_sensor_data(data, torque_sensors[sensor_key])

        if real_torque is not None:
            real_torque = smooth_data(real_torque, sensor_key)
            torque_data_all[sensor_key].append(real_torque)
            print(f"{sensor_key} Real Torque: {real_torque}")

    # Rendering
    viewport = mujoco.MjrRect(0, 0, glfw.get_framebuffer_size(window)[0], glfw.get_framebuffer_size(window)[1])
    mujoco.mjv_updateScene(model, data, opt, None, cam, mujoco.mjtCatBit.mjCAT_ALL, scn)

    if not grasp_successful:
        # Contact checking
        all_fingers_contacted, contact_info = check_contact(model, data, "object", hand_body_names)
        if all_fingers_contacted:
            print("Grasp successful!")
            for finger, pos in contact_info.items():
                print(f"{finger} contact position: {pos}")
            grasp_successful = True

    mujoco.mjr_render(viewport, scn, con)


    width, height = glfw.get_framebuffer_size(window)
    framebuffer = np.zeros((height, width, 3), dtype=np.uint8)
    mujoco.mjr_readPixels(framebuffer, None, viewport, con)
    video_writer.append_data(np.flipud(framebuffer))  

    glfw.swap_buffers(window)
    glfw.poll_events()

    # Calculate and display frame rate
    end_time = time.time()
    frame_time = end_time - start_time
    print(f"Frame Time: {frame_time:.4f} seconds, FPS: {1.0 / frame_time:.2f}")

# End video recording
video_writer.close()
glfw.terminate()
print(f"Video saved to {video_path}")

# Plot and save an image of the moment data
plt.figure(figsize=(12, 8))
for key, data in torque_data_all.items():
    plt.plot(data, label=key)
plt.xlabel('Time Steps')
plt.ylabel('Torque')
plt.title('Real-time Torque Data During Grasping with PID Controller')
plt.legend()
plt.savefig('torque_data(PID Controller).png')
plt.show()

print(f"Torque data saved to torque_data(PID Controller).png")

