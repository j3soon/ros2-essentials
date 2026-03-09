# Table of Contents

* [stretch\_urdf.urdf\_utils](#stretch_urdf.urdf_utils)
  * [get\_latest\_urdf](#stretch_urdf.urdf_utils.get_latest_urdf)
  * [clip\_joint\_limits](#stretch_urdf.urdf_utils.clip_joint_limits)
  * [make\_joints\_rigid](#stretch_urdf.urdf_utils.make_joints_rigid)
  * [merge\_arm](#stretch_urdf.urdf_utils.merge_arm)
  * [add\_virtual\_rotary\_joint](#stretch_urdf.urdf_utils.add_virtual_rotary_joint)
  * [generate\_urdf\_from\_robot](#stretch_urdf.urdf_utils.generate_urdf_from_robot)
  * [generate\_ik\_urdfs](#stretch_urdf.urdf_utils.generate_ik_urdfs)

<a id="stretch_urdf.urdf_utils"></a>

# stretch\_urdf.urdf\_utils

<a id="stretch_urdf.urdf_utils.get_latest_urdf"></a>

## get\_latest\_urdf

```python
def get_latest_urdf()
```

Fetches the latest calibrated URDF from the calibration directory.

**Returns**

* `str`: Absolute filepath to the latest calibrated URDF.

<a id="stretch_urdf.urdf_utils.clip_joint_limits"></a>

## clip\_joint\_limits

```python
def clip_joint_limits(robot, use_original_limits=True)
```

Enables more conservative joint limits to be set than in the
original URDF.

If these limits are outside the originally permitted range,
the original range is used. Joint limits. Where these limits
have a value of None, the original limit is used.

**Arguments**

* **robot** (`urdf_parser_py.urdf.Robot`): a manipulable URDF representation
* **use_original_limits** (`bool`): don't impose any additional limits

**Returns**

* `urdf_parser_py.urdf.Robot`: modified URDF where joint limits are clipped

<a id="stretch_urdf.urdf_utils.make_joints_rigid"></a>

## make\_joints\_rigid

```python
def make_joints_rigid(robot, ignore_joints=None)
```

Change any joint that should be immobile for end effector IK
into a fixed joint.

**Arguments**

* **robot** (`urdf_parser_py.urdf.Robot`): a manipulable URDF representation
* **ignore_joints** (`list(str) or None`): which joints to keep as-is

**Returns**

* `urdf_parser_py.urdf.Robot`: modified URDF where joints are "fixed"

<a id="stretch_urdf.urdf_utils.merge_arm"></a>

## merge\_arm

```python
def merge_arm(robot)
```

Replace telescoping arm with a single prismatic joint,
which makes end-effector IK computation easier.

**Arguments**

* **robot** (`urdf_parser_py.urdf.Robot`): a manipulable URDF representation

**Returns**

* `urdf_parser_py.urdf.Robot`: modified URDF with single arm joint

<a id="stretch_urdf.urdf_utils.add_virtual_rotary_joint"></a>

## add\_virtual\_rotary\_joint

```python
def add_virtual_rotary_joint(robot)
```

Add virtual rotary joint for mobile base.

**Arguments**

* **robot** (`urdf_parser_py.urdf.Robot`): a manipulable URDF representation

**Returns**

* `urdf_parser_py.urdf.Robot`: modified URDF with mobile base rotation joint

<a id="stretch_urdf.urdf_utils.generate_urdf_from_robot"></a>

## generate\_urdf\_from\_robot

```python
def generate_urdf_from_robot(robot, app_name, description=None)
```

Renders a `robot` URDF object out to a file in the /tmp
folder. The file will be unique to your application given
the `app_name` isn't the same as other applications.

This enables you to safety generate URDFs on-the-fly
to be used by your app. E.g. `generate_ik_urdfs()` uses
this method to generate "calibrated" inverse kinematics
URDFs, so each robot's unique backlash and skew parameters
are baked into the IK calculations.

**Arguments**

* **robot** (`urdf_parser_py.urdf.Robot`): the URDF representation to render out to a file
* **app_name** (`str`): the name of your application
* **description** (`str or None`): further description of the URDF

**Returns**

* `str`: filepath of the generated URDF

<a id="stretch_urdf.urdf_utils.generate_ik_urdfs"></a>

## generate\_ik\_urdfs

```python
def generate_ik_urdfs(app_name, rigid_wrist_urdf=True)
```

Generates URDFs for IK packages. The latest calibrated
URDF is used as a starting point, then these modifications
are applied:
  1. Clip joint limits
  2. Make non-IK joints rigid
  3. Merge arm joints
  4. Add virtual rotary base joint
  5. (optionally) Make wrist joints rigid

**Arguments**

* **app_name** (`str`): the name of your application
* **rigid_wrist_urdf** (`bool or None`): whether to also generate a IK URDF with a fixed dex wrist

**Returns**

* `list(str)`: one or two filepaths, depending on `rigid_wrist_urdf`,
to the generated URDFs. The first element will be the
full IK version, and the second will be the rigid
wrist version.

