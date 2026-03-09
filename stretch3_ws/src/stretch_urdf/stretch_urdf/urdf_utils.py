import pathlib
import numpy as np
import stretch_body.hello_utils as hu
from urdf_parser_py import urdf as ud
from typing import Dict, Optional, Tuple


def get_latest_urdf():
    """
    Fetches the latest calibrated URDF from the calibration directory.

    Returns
    -------
    str
        Absolute filepath to the latest calibrated URDF.
    """
    try:
        fleet_dir = hu.get_fleet_directory()
    except:
        raise FileNotFoundError("Stretch data directory doesn't exist") from None
    calibration_dir = pathlib.Path(fleet_dir) / "exported_urdf"
    if not calibration_dir.is_dir():
        raise FileNotFoundError("URDF calibration directory doesn't exist")
    urdf_path = calibration_dir / "stretch.urdf"
    if not urdf_path.is_file():
        raise FileNotFoundError("URDF doesn't exist")
    urdf_filename = str(urdf_path.absolute())
    return urdf_filename


def clip_joint_limits(robot, use_original_limits=True):
    """
    Enables more conservative joint limits to be set than in the
    original URDF.

    If these limits are outside the originally permitted range,
    the original range is used. Joint limits. Where these limits
    have a value of None, the original limit is used.

    Parameters
    ----------
    robot : urdf_parser_py.urdf.Robot
        a manipulable URDF representation
    use_original_limits : bool
        don't impose any additional limits

    Returns
    -------
    urdf_parser_py.urdf.Robot
        modified URDF where joint limits are clipped
    """
    ik_joint_limits: Dict[str, Tuple[Optional[float], Optional[float]]] = {}
    if use_original_limits:
        ik_joint_limits = {
            "joint_mobile_base_translation": (None, None),
            "joint_mobile_base_rotation": (None, None),
            "joint_lift": (None, None),
            "joint_arm_l0": (None, None),
            "joint_wrist_yaw": (None, None),
            "joint_wrist_pitch": (-0.8 * (np.pi / 2.0), None), # Beware of gimbal lock if joint_wrist_pitch is too close to -90 deg
            "joint_wrist_roll": (None, None),
        }
    else:
        ik_joint_limits = {
            "joint_mobile_base_translation": (-0.25, 0.25),
            "joint_mobile_base_rotation": (-(np.pi / 2.0), np.pi / 2.0),
            "joint_lift": (0.01, 1.1),
            "joint_arm_l0": (0.01, 0.48),
            "joint_wrist_yaw": (-(np.pi / 4.0), np.pi),
            "joint_wrist_pitch": (-0.9 * (np.pi / 2.0), np.pi / 20.0),
            "joint_wrist_roll": (-(np.pi / 2.0), np.pi / 2.0),
        }

    for j in ik_joint_limits:
        joint = robot.joint_map.get(j, None)
        if joint is not None:
            original_upper = joint.limit.upper
            requested_upper = ik_joint_limits[j][1]
            if requested_upper is not None:
                new_upper = min(requested_upper, original_upper)
                robot.joint_map[j].limit.upper = new_upper

            original_lower = joint.limit.lower
            requested_lower = ik_joint_limits[j][0]
            if requested_lower is not None:
                new_lower = max(requested_lower, original_lower)
                robot.joint_map[j].limit.lower = new_lower


def make_joints_rigid(robot, ignore_joints=None):
    """
    Change any joint that should be immobile for end effector IK
    into a fixed joint.

    Parameters
    ----------
    robot : urdf_parser_py.urdf.Robot
        a manipulable URDF representation
    ignore_joints : list(str) or None
        which joints to keep as-is

    Returns
    -------
    urdf_parser_py.urdf.Robot
        modified URDF where joints are "fixed"
    """
    if ignore_joints is None:
        ignore_joints = []

    for j in robot.joint_map.keys():
        if j not in ignore_joints:
            joint = robot.joint_map[j]
            joint.type = "fixed"


def merge_arm(robot):
    """
    Replace telescoping arm with a single prismatic joint,
    which makes end-effector IK computation easier.

    Parameters
    ----------
    robot : urdf_parser_py.urdf.Robot
        a manipulable URDF representation

    Returns
    -------
    urdf_parser_py.urdf.Robot
        modified URDF with single arm joint
    """
    all_arm_joints = [
        "joint_arm_l4",
        "joint_arm_l3",
        "joint_arm_l2",
        "joint_arm_l1",
        "joint_arm_l0",
    ]
    prismatic_arm_joints = all_arm_joints[1:]
    removed_arm_joints = all_arm_joints[1:-1]
    near_proximal_arm_joint = robot.joint_map[all_arm_joints[1]]
    distal_arm_joint = robot.joint_map[all_arm_joints[-1]]

    # Calculate aggregate joint characteristics
    xyz_total = np.array([0.0, 0.0, 0.0])
    limit_upper_total = 0.0
    for j in prismatic_arm_joints:
        joint = robot.joint_map[j]
        xyz_total = xyz_total + joint.origin.xyz
        limit_upper_total = limit_upper_total + joint.limit.upper

    # Directly connect the proximal and distal parts of the arm
    distal_arm_joint.parent = near_proximal_arm_joint.parent

    # Make the distal prismatic joint act like the full arm
    distal_arm_joint.origin.xyz = xyz_total
    distal_arm_joint.limit.upper = limit_upper_total

    # Mark the eliminated joints as "fixed"
    for j in removed_arm_joints:
        joint = robot.joint_map[j]
        joint.type = "fixed"


def add_virtual_base_joints(robot, base_rotation):
    """
    Add virtual rotary joint for mobile base.

    Parameters
    ----------
    robot : urdf_parser_py.urdf.Robot
        a manipulable URDF representation

    Returns
    -------
    urdf_parser_py.urdf.Robot
        modified URDF with mobile base rotation joint
    """
    link_virtual_base = ud.Link(
        name="virtual_base", visual=None, inertial=None, collision=None, origin=None
    )

    origin_rotary = ud.Pose(xyz=[0, 0, 0], rpy=[0, 0, 0])
    limit_rotary = ud.JointLimit(effort=10, velocity=1, lower=-np.pi, upper=np.pi)
    limit_prismatic = ud.JointLimit(effort=10, velocity=1, lower=-0.5, upper=0.5)

    joint_mobile_base_translation = ud.Joint(
        name="joint_mobile_base_translation",
        parent="virtual_base",
        child="base_link",
        joint_type="prismatic",
        axis=[1, 0, 0],
        origin=origin_rotary,
        limit=limit_prismatic,
        dynamics=None,
        safety_controller=None,
        calibration=None,
        mimic=None,
    )
    joint_mobile_base_rotation = ud.Joint(
        name="joint_mobile_base_rotation",
        parent="virtual_base",
        child="base_link",
        joint_type="revolute",
        axis=[0, 0, 1],
        origin=origin_rotary,
        limit=limit_rotary,
        dynamics=None,
        safety_controller=None,
        calibration=None,
        mimic=None,
    )
    robot.add_link(link_virtual_base)
    if base_rotation:
        robot.add_joint(joint_mobile_base_rotation)
    else:
        robot.add_joint(joint_mobile_base_translation)

def generate_urdf_from_robot(robot, app_name, description=None):
    """
    Renders a `robot` URDF object out to a file in the /tmp
    folder. The file will be unique to your application given
    the `app_name` isn't the same as other applications.

    This enables you to safety generate URDFs on-the-fly
    to be used by your app. E.g. `generate_ik_urdfs()` uses
    this method to generate "calibrated" inverse kinematics
    URDFs, so each robot's unique backlash and skew parameters
    are baked into the IK calculations.

    Parameters
    ----------
    robot : urdf_parser_py.urdf.Robot
        the URDF representation to render out to a file
    app_name : str
        the name of your application
    description : str or None
        further description of the URDF

    Returns
    -------
    str
        filepath of the generated URDF
    """
    if description is None:
        description = "custom"

    def save_urdf(robot, file_name):
        urdf_string = robot.to_xml_string()
        with open(file_name, "w") as fid:
            fid.write(urdf_string)

    filename = f'/tmp/{app_name}_{description}.urdf'
    save_urdf(robot, filename)

    return filename


def generate_ik_urdfs(app_name, rigid_wrist_urdf=True, base_rotation=True):
    """
    Generates URDFs for IK packages. The latest calibrated
    URDF is used as a starting point, then these modifications
    are applied:
      1. Clip joint limits
      2. Make non-IK joints rigid
      3. Merge arm joints
      4. Add virtual rotary base joint
      5. (optionally) Make wrist joints rigid

    Parameters
    ----------
    app_name : str
        the name of your application
    rigid_wrist_urdf : bool or None
        whether to also generate a IK URDF with a fixed dex wrist

    Returns
    -------
    list(str)
        one or two filepaths, depending on `rigid_wrist_urdf`,
        to the generated URDFs. The first element will be the
        full IK version, and the second will be the rigid
        wrist version.
    """

    robot = ud.Robot.from_xml_file(get_latest_urdf())
    clip_joint_limits(robot)

    ignore_joints = [
        "joint_lift",
        "joint_arm_l0",
        "joint_arm_l1",
        "joint_arm_l2",
        "joint_arm_l3",
        "joint_wrist_yaw",
        "joint_wrist_pitch",
        "joint_wrist_roll",
    ]
    make_joints_rigid(robot, ignore_joints)

    merge_arm(robot)

    add_virtual_base_joints(robot, base_rotation)

    ret = []
    description = 'base_rotation_ik' if base_rotation else 'base_translation_ik' 
    fpath = generate_urdf_from_robot(robot, app_name, description)
    ret.append(fpath)

    if rigid_wrist_urdf:
        ignore_joints = [
            "joint_mobile_base_translation",
            "joint_mobile_base_rotation",
            "joint_lift",
            "joint_arm_l0",
        ]
        make_joints_rigid(robot, ignore_joints)

        fpath = generate_urdf_from_robot(robot, app_name, 'base_rotation_ik_with_fixed_wrist')
        ret.append(fpath)

    return ret


if __name__ == "__main__":
    print(generate_ik_urdfs('stretch_web_teleop'))
    with open('/tmp/stretch_web_teleop_base_rotation_ik.urdf', 'r') as fid:
        print(fid.read())
