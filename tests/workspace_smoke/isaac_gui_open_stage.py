#!/usr/bin/env python3
"""Open a USD stage in Isaac Sim GUI for doc-demo screenshot proof."""

from __future__ import annotations

import argparse
import asyncio
import os

import carb
import omni.kit.app
import omni.timeline
import omni.usd


_STAGE_TASK: asyncio.Future | None = None


def look_at_transform(eye, target, up):
    from pxr import Gf

    return Gf.Matrix4d().SetLookAt(
        Gf.Vec3d(*eye),
        Gf.Vec3d(*target),
        Gf.Vec3d(*up),
    ).GetInverse()


def frame_prim(stage, prim_path: str) -> None:
    from pxr import Gf, Usd, UsdGeom, UsdLux

    prim = stage.GetPrimAtPath(prim_path)
    bbox_cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_, UsdGeom.Tokens.render])
    bbox = bbox_cache.ComputeWorldBound(prim).ComputeAlignedBox()
    if bbox.IsEmpty():
        return

    center = bbox.GetMidpoint()
    size = bbox.GetSize()
    radius = max(float(size[0]), float(size[1]), float(size[2]), 0.5)
    camera_path = "/World/DocDemoCamera" if stage.GetPrimAtPath("/World").IsValid() else "/DocDemoCamera"
    light_path = "/World/DocDemoKeyLight" if stage.GetPrimAtPath("/World").IsValid() else "/DocDemoKeyLight"

    light = UsdLux.DistantLight.Define(stage, light_path)
    light.CreateIntensityAttr(900.0)
    UsdGeom.XformCommonAPI(light).SetRotate((50.0, 0.0, 35.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)

    camera = UsdGeom.Camera.Define(stage, camera_path)
    camera.CreateFocalLengthAttr(24.0)
    eye = (
        float(center[0]) + radius * 1.8,
        float(center[1]) - radius * 2.2,
        float(center[2]) + radius * 1.4,
    )
    camera.AddTransformOp().Set(
        look_at_transform(
            eye=eye,
            target=(float(center[0]), float(center[1]), float(center[2])),
            up=(0.0, 0.0, 1.0),
        )
    )

    from omni.kit.viewport.utility import get_active_viewport

    viewport = get_active_viewport()
    if viewport:
        viewport.camera_path = camera_path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--path", default=os.environ.get("ISAAC_GUI_STAGE_PATH"), help="USD/USDA stage to open.")
    parser.add_argument(
        "--expected-prim",
        default=os.environ.get("ISAAC_GUI_EXPECTED_PRIM"),
        help="Robot prim that must exist after opening the stage.",
    )
    parser.add_argument(
        "--play",
        action="store_true",
        default=os.environ.get("ISAAC_GUI_PLAY", "").lower() in {"1", "true", "yes"},
        help="Start the timeline after opening the stage.",
    )
    parser.add_argument(
        "--settle-frames",
        type=int,
        default=int(os.environ.get("ISAAC_GUI_SETTLE_FRAMES", "180")),
        help="Frames to wait after opening.",
    )
    args, _ = parser.parse_known_args()
    if not args.path:
        parser.error("--path or ISAAC_GUI_STAGE_PATH is required")
    return args


async def main_async() -> None:
    args = parse_args()
    carb.settings.get_settings().set("/exts/omni.kit.notification_manager/disable_notifications", True)
    context = omni.usd.get_context()
    success, error = await context.open_stage_async(args.path)
    if not success:
        print(f"ISAAC_GUI_STAGE_FAILED path={args.path} error={error}", flush=True)
        carb.log_error(f"Failed to open stage {args.path}: {error}")
        return

    print(f"ISAAC_GUI_STAGE_OPENED path={args.path}", flush=True)

    app = omni.kit.app.get_app()
    stage = context.get_stage()
    if args.expected_prim:
        prim = stage.GetPrimAtPath(args.expected_prim) if stage else None
        if prim is None or not prim.IsValid():
            print(f"ISAAC_GUI_EXPECTED_PRIM_MISSING path={args.expected_prim}", flush=True)
        else:
            if not prim.IsLoaded():
                stage.Load(args.expected_prim)
                for _ in range(30):
                    await app.next_update_async()
            child_count = sum(1 for _ in prim.GetChildren())
            print(
                "ISAAC_GUI_EXPECTED_PRIM_OK "
                f"path={args.expected_prim} loaded={prim.IsLoaded()} children={child_count}",
                flush=True,
            )
            frame_prim(stage, args.expected_prim)

    if args.play:
        omni.timeline.get_timeline_interface().play()
        print("ISAAC_GUI_TIMELINE_PLAYING", flush=True)

    for _ in range(max(args.settle_frames, 0)):
        await app.next_update_async()


def main() -> None:
    global _STAGE_TASK
    _STAGE_TASK = asyncio.ensure_future(main_async())


main()
