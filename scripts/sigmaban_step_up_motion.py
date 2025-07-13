import placo
import numpy as np
import time
import json

from ischedule import schedule, run_loop
from placo_utils.visualization import robot_viz, point_viz, robot_frame_viz
from scipy.spatial.transform import Rotation as R

feet_spacing = 0.14

robot = placo.HumanoidRobot(
    "open_duck_reference_motion_generator/robots/sigmaban2019/sigmaban2019.urdf"
)
robot.set_joint_limits("left_knee", 0.01, 2)
robot.set_joint_limits("right_knee", 0.01, 2)


T_world_left = np.eye(4)
T_world_left[:3, 3][1] += feet_spacing / 2.0
# Placing the left foot in world origin
robot.set_T_world_frame("left_foot", T_world_left)
robot.update_kinematics()

solver = placo.KinematicsSolver(robot)

# Retrieving initial position of the feet, com and trunk orientation
T_world_left = robot.get_T_world_frame("left_foot")
T_world_right = T_world_left.copy()
T_world_right[:3][1] -= feet_spacing

# Creating the viewer
viz = robot_viz(robot)

# Trunk
com_task = solver.add_com_task(np.array([0.0, 0.0, 0.3]))
com_task.configure("com", "soft", 1.0)

trunk_orientation_task = solver.add_orientation_task("trunk", np.eye(3))
trunk_orientation_task.configure("trunk_orientation", "soft", 1.0)

# Keep left and right foot on the floor
left_foot_task = solver.add_frame_task("left_foot", T_world_left)
left_foot_task.configure("left_foot", "soft", 1.0, 1.0)

right_foot_task = solver.add_frame_task("right_foot", T_world_right)
right_foot_task.configure("right_foot", "soft", 1.0, 1.0)

# Creating a very basic lateral swing and foot rise trajectory
# t, x, dx
left_foot_z_traj = placo.CubicSpline()
left_foot_z_traj.add_point(0.0, 0.0, 0.0)
left_foot_z_traj.add_point(0.5, 0.16, 0.0)
left_foot_z_traj.add_point(1.0, 0.15, 0.0)
left_foot_z_traj.add_point(2.0, 0.15, 0.0)
left_foot_z_traj.add_point(3.0, 0.15, 0.0)

left_foot_x_traj = placo.CubicSpline()
left_foot_x_traj.add_point(0.0, 0.0, 0.0)
left_foot_x_traj.add_point(1.0, 0.2, 0.0)
left_foot_x_traj.add_point(2.0, 0.2, 0.0)
left_foot_x_traj.add_point(3.0, 0.2, 0.0)

right_foot_z_traj = placo.CubicSpline()
right_foot_z_traj.add_point(0.0, 0.0, 0.0)
right_foot_z_traj.add_point(1.0, 0.0, 0.0)
right_foot_z_traj.add_point(1.5, 0.16, 0.0)
right_foot_z_traj.add_point(2.0, 0.16, 0.0)
right_foot_z_traj.add_point(3.0, 0.16, 0.0)

right_foot_x_traj = placo.CubicSpline()
right_foot_x_traj.add_point(0.0, 0.0, 0.0)
right_foot_x_traj.add_point(1.0, 0.0, 0.0)
right_foot_x_traj.add_point(2.0, 0.2, 0.0)
right_foot_x_traj.add_point(3.0, 0.2, 0.0)

com_x_traj = placo.CubicSpline()
com_x_traj.add_point(0.0, 0.0, 0.0)
com_x_traj.add_point(1.0, 0.08, 0.0)
com_x_traj.add_point(2.0, 0.20, 0.0)
com_x_traj.add_point(3.0, 0.20, 0.0)

com_y_traj = placo.CubicSpline()
com_y_traj.add_point(0.0, 0.0, 0.0)
com_y_traj.add_point(0.5, -0.08, 0.0)
com_y_traj.add_point(1.0, 0.00, 0.0)
com_y_traj.add_point(1.5, 0.08, 0.0)
com_y_traj.add_point(2.0, 0.0, 0.0)
com_y_traj.add_point(3.0, 0.0, 0.0)


com_z_traj = placo.CubicSpline()
com_z_traj.add_point(0.0, 0.3, 0.0)
com_z_traj.add_point(1.0, 0.33, 0.0)
com_z_traj.add_point(1.5, 0.45, 0.0)
com_z_traj.add_point(2.0, 0.45, 0.0)
com_z_traj.add_point(3.0, 0.45, 0.0)


joints= [
"head_yaw",
"head_pitch",
"left_shoulder_pitch",
"left_shoulder_roll",
"left_elbow",
"right_shoulder_pitch",
"right_shoulder_roll",
"right_elbow",
"left_hip_yaw",
"left_hip_roll",
"left_hip_pitch",
"left_knee",
"left_ankle_pitch",
"left_ankle_roll",
"right_hip_yaw",
"right_hip_roll",
"right_hip_pitch",
"right_knee",
"right_ankle_pitch",
"right_ankle_roll"
]

# Regularization task
posture_regularization_task = solver.add_joints_task()
posture_regularization_task.set_joints({dof: 0.0 for dof in joints[:8]})
posture_regularization_task.configure("reg", "soft", 1)

solver.enable_joint_limits(True)
solver.enable_velocity_limits(True)

t = 0
dt = 0.01
last = 0
solver.dt = dt
start_t = time.time()
robot.update_kinematics()


FPS = 50
episode = {
    "LoopMode": "Wrap",
    "FPS": FPS,
    "FrameDuration": np.around(1 / FPS, 4),
    "EnableCycleOffsetPosition": True,
    "EnableCycleOffsetRotation": False,
    "Joints": [],
    "Vel_x": [],
    "Vel_y": [],
    "Yaw": [],
    "Placo": [],
    "Frame_offset": [],
    "Frame_size": [],
    "Frames": [],
    "MotionWeight": 1,
}


def compute_angular_velocity(quat, prev_quat, dt):
    # Convert quaternions to scipy Rotation objects
    if prev_quat is None:
        prev_quat = quat
    r1 = R.from_quat(quat)  # Current quaternion
    r0 = R.from_quat(prev_quat)  # Previous quaternion

    # Compute relative rotation: r_rel = r0^(-1) * r1
    r_rel = r0.inv() * r1

    # Convert relative rotation to axis-angle
    axis, angle = r_rel.as_rotvec(), np.linalg.norm(r_rel.as_rotvec())

    # Angular velocity (in radians per second)
    angular_velocity = axis * (angle / dt)

    return list(angular_velocity)


prev_initialized = False
added_frame_info = False

last_rec = t

class RoundingFloat(float):
    __repr__ = staticmethod(lambda x: format(x, ".5f"))

while True:

    # Updating the target
    t_mod = t % 3.0
    target = left_foot_task.position().target_world
    target[0] = left_foot_x_traj.pos(t_mod)
    target[2] = left_foot_z_traj.pos(t_mod)
    left_foot_task.position().target_world = target

    target = right_foot_task.position().target_world
    target[0] = right_foot_x_traj.pos(t_mod)
    target[2] = right_foot_z_traj.pos(t_mod)
    right_foot_task.position().target_world = target

    # Updating the com target with lateral sinusoidal trajectory
    # com_task.target_world = np.array([0.0, 0, 0.2])
    com_task.target_world = np.array(
        [com_x_traj.pos(t_mod), com_y_traj.pos(t_mod), com_z_traj.pos(t_mod)]
    )
    # com_task.target_world = np.array([0.0, -np.sin(t * np.pi) * 0.05, 0.3])

    solver.solve(True)
    robot.update_kinematics()

    viz.display(robot.state.q)
    robot_frame_viz(robot, "trunk")
    robot_frame_viz(robot, "camera")
    point_viz("com", robot.com_world(), radius=0.025, color=0xAAAAAA)

    if t - last_rec > 1.0 / FPS:
        T_world_fbase = robot.get_T_world_fbase()
        root_position = list(T_world_fbase[:3, 3])
        root_orientation_quat = list(R.from_matrix(T_world_fbase[:3, :3]).as_quat())

        angles = {joint: robot.get_joint(joint) for joint in joints}
        joints_positions = list(angles.values())

        T_world_leftFoot = robot.get_T_world_left()
        T_world_rightFoot = robot.get_T_world_right()

        T_body_leftFoot = np.linalg.inv(T_world_fbase) @ T_world_leftFoot
        T_body_rightFoot = np.linalg.inv(T_world_fbase) @ T_world_rightFoot

        left_toe_pos = list(T_body_leftFoot[:3, 3])
        right_toe_pos = list(T_body_rightFoot[:3, 3])

        if not prev_initialized:
            prev_root_position = root_position.copy()
            prev_root_orientation_quat = root_orientation_quat.copy()
            prev_left_toe_pos = left_toe_pos.copy()
            prev_right_toe_pos = right_toe_pos.copy()
            prev_joints_positions = joints_positions.copy()
            prev_initialized = True

        world_linear_vel = list(
            (np.array(root_position) - np.array(prev_root_position)) / (1 / FPS)
        )

        world_angular_vel = compute_angular_velocity(
            root_orientation_quat, prev_root_orientation_quat, (1 / FPS)
        )

        joints_vel = list(
            (np.array(joints_positions) - np.array(prev_joints_positions)) / (1 / FPS)
        )

        left_toe_vel = list(
            (np.array(left_toe_pos) - np.array(prev_left_toe_pos)) / (1 / FPS)
        )
        right_toe_vel = list(
            (np.array(right_toe_pos) - np.array(prev_right_toe_pos)) / (1 / FPS)
        )

        episode["Frames"].append(
            root_position
            + root_orientation_quat
            + joints_positions
            + left_toe_pos
            + right_toe_pos
            + world_linear_vel
            + world_angular_vel
            + joints_vel
            + left_toe_vel
            + right_toe_vel
            + [0, 0]
        )
        if not added_frame_info:
            added_frame_info = True
            offset = 0
            offset_root_pos = offset
            offset = offset + len(root_position)
            offset_root_quat = offset
            offset = offset + len(root_orientation_quat)
            offset_joints_pos = offset
            offset = offset + len(joints_positions)
            offset_left_toe_pos = offset
            offset = offset + len(left_toe_pos)
            offset_right_toe_pos = offset
            offset = offset + len(right_toe_pos)
            offset_world_linear_vel = offset
            offset = offset + len(world_linear_vel)
            offset_world_angular_vel = offset
            offset = offset + len(world_angular_vel)
            offset_joints_vel = offset
            offset = offset + len(joints_vel)
            offset_left_toe_vel = offset
            offset = offset + len(left_toe_vel)
            offset_right_toe_vel = offset
            offset = offset + len(right_toe_vel)
            offset_foot_contacts = offset
            offset = offset + len([0, 0])

            episode["Joints"] = list(joints)
            episode["Frame_offset"].append(
                {
                    "root_pos": offset_root_pos,
                    "root_quat": offset_root_quat,
                    "joints_pos": offset_joints_pos,
                    "left_toe_pos": offset_left_toe_pos,
                    "right_toe_pos": offset_right_toe_pos,
                    "world_linear_vel": offset_world_linear_vel,
                    "world_angular_vel": offset_world_angular_vel,
                    "joints_vel": offset_joints_vel,
                    "left_toe_vel": offset_left_toe_vel,
                    "right_toe_vel": offset_right_toe_vel,
                    "foot_contacts": offset_foot_contacts,
                }
            )
            episode["Frame_size"].append(
                {
                    "root_pos": len(root_position),
                    "root_quat": len(root_orientation_quat),
                    "joints_pos": len(joints_positions),
                    "left_toe_pos": len(left_toe_pos),
                    "right_toe_pos": len(right_toe_pos),
                    "world_linear_vel": len(world_linear_vel),
                    "world_angular_vel": len(world_angular_vel),
                    "joints_vel": len(joints_vel),
                    "left_toe_vel": len(left_toe_vel),
                    "right_toe_vel": len(right_toe_vel),
                    "foot_contacts": len([0, 0]),
                }
            )

        prev_root_position = root_position.copy()
        prev_root_orientation_quat = root_orientation_quat.copy()
        prev_left_toe_pos = left_toe_pos.copy()
        prev_right_toe_pos = right_toe_pos.copy()
        print(t)
        if t> 3.0:
            episode["Placo"] = {
                "period": 3.0,
            }
            with open("parkour.json","w") as f:
                json.encoder.c_make_encoder = None
                json.encoder.float = RoundingFloat
                json.dump(episode, f, indent=4)
            exit()
        last_rec = t

    t += dt
