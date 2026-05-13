# Isaac Sim / Isaac Lab examples for ur5_ws

Minimal scripts that load a Universal Robots UR5 or UR5e in Isaac Sim and Isaac Lab.

| Script | Runtime | Purpose |
| --- | --- | --- |
| `spawn_ur.py` | Isaac Sim standalone | Load UR5/UR5e USD, run physics, render. Good first sanity check. |
| `ur_isaaclab_minimal.py` | Isaac Lab | Load UR5/UR5e as an `Articulation` with `InteractiveScene`. Scaffold for RL / Reach / Pick. |

## How to run

Inside the `ur5_ws` container (Isaac Sim and Isaac Lab installed via the build args in `docker/compose.yaml`):

```sh
# Convenience wrappers in ur5_ws/scripts/
./scripts/isaac_sim_ur5.sh
./scripts/isaac_sim_ur5e.sh

# Or directly:
${ISAACSIM_PYTHON_EXE} scripts/isaac/spawn_ur.py --ur_type ur5e
~/IsaacLab/isaaclab.sh -p scripts/isaac/ur_isaaclab_minimal.py --ur_type ur5
```

Pass `--headless` to the Isaac Sim script for offscreen rendering (e.g., on a cluster without a desktop).

## Extending to RL / manipulation tasks

`ur_isaaclab_minimal.py` is intentionally bare. To turn it into a full Reach or Pick task, copy the UR10 manager-based config in Isaac Lab — typically at:

```
$ISAACLAB_PATH/source/isaaclab_tasks/isaaclab_tasks/manager_based/manipulation/reach/config/ur_10/
```

Swap the `usd_path` and joint init values for UR5/UR5e (joint names are identical across the UR family, so the joint-name lists in the config don't need to change).
