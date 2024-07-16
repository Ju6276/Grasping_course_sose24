import mujoco

    # Get the name of a body by its ID
def get_body_name(model, body_id):
    name_start = model.name_bodyadr[body_id]
    name_end = model.names.find(b'\0', name_start)
    body_name = model.names[name_start:name_end].decode('utf-8')
    return body_name

    # Get the position of a body by its name
def get_body_position(model, data, body_name):
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    return data.xpos[body_id]

    # Print the names and IDs of all bodies in the model
def print_all_bodies(model):
    print("All bodies in the model:")
    for body_id in range(model.nbody):
        body_name = get_body_name(model, body_id)
        print(f"Body ID {body_id}: {body_name}")

    # Get the IDs of joints based on their names
def get_joint_ids(model, joint_goals):
    joint_ids = {name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in joint_goals.keys()}
    return joint_ids

    # Get the IDs of actuators based on joint names
def get_actuator_ids(model, joint_goals):
    actuator_ids = {name.replace('j', 'a'): mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name.replace('j', 'a')) for name in joint_goals.keys()}
    return actuator_ids
