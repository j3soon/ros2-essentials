#!/usr/bin/env python3
"""Render a tiny Isaac Sim scene and write a PNG proof artifact."""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Create a deterministic Isaac Sim scene and capture a viewport PNG."
    )
    parser.add_argument("--output", required=True, type=Path, help="PNG output path.")
    parser.add_argument("--label", default="workspace", help="Label printed in the log.")
    parser.add_argument(
        "--stage",
        help="Optional USD/USDA stage path to open instead of creating the built-in smoke scene.",
    )
    parser.add_argument(
        "--play",
        action="store_true",
        help="Start the Isaac timeline before capture. Useful for documented simulator scenes.",
    )
    parser.add_argument("--width", type=int, default=640, help="Capture width.")
    parser.add_argument("--height", type=int, default=480, help="Capture height.")
    parser.add_argument(
        "--cpu-threads",
        type=int,
        default=16,
        help="Maximum Isaac Sim worker threads. Defaults to 16.",
    )
    parser.add_argument("--settle-frames", type=int, default=60, help="Frames rendered before capture.")
    parser.add_argument("--capture-frames", type=int, default=180, help="Frames allowed for capture completion.")
    return parser.parse_args()


def look_at_transform(eye, target, up):
    from pxr import Gf

    # USD cameras look down local -Z, so use the inverse view matrix as the prim transform.
    return Gf.Matrix4d().SetLookAt(
        Gf.Vec3d(*eye),
        Gf.Vec3d(*target),
        Gf.Vec3d(*up),
    ).GetInverse()


def create_scene(width: int, height: int) -> str:
    import carb.settings
    import omni.usd
    from pxr import Gf, Sdf, UsdGeom, UsdLux

    settings = carb.settings.get_settings()
    settings.set("/persistent/app/viewport/displayOptions", 0)

    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    cube = UsdGeom.Cube.Define(stage, "/World/SmokeCube")
    cube.CreateSizeAttr(1.0)
    UsdGeom.XformCommonAPI(cube).SetTranslate((0.0, 0.0, 0.55))
    cube.GetPrim().CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set(
        [Gf.Vec3f(0.05, 0.38, 0.95)]
    )

    ground = UsdGeom.Cube.Define(stage, "/World/Ground")
    ground.CreateSizeAttr(1.0)
    UsdGeom.XformCommonAPI(ground).SetScale((4.0, 4.0, 0.025))
    UsdGeom.XformCommonAPI(ground).SetTranslate((0.0, 0.0, -0.025))
    ground.GetPrim().CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set(
        [Gf.Vec3f(0.72, 0.72, 0.66)]
    )

    light = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
    light.CreateIntensityAttr(650.0)
    UsdGeom.XformCommonAPI(light).SetRotate((45.0, 0.0, 35.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)

    camera = UsdGeom.Camera.Define(stage, "/World/Camera")
    camera.CreateFocalLengthAttr(28.0)
    camera.AddTransformOp().Set(
        look_at_transform(
            eye=(3.0, -4.0, 2.4),
            target=(0.0, 0.0, 0.45),
            up=(0.0, 0.0, 1.0),
        )
    )

    from omni.kit.viewport.utility import get_active_viewport

    viewport = get_active_viewport()
    if viewport:
        viewport.camera_path = "/World/Camera"
        viewport.resolution = (width, height)
    return "/World/Camera"


def open_stage_scene(stage_path: str, width: int, height: int, play: bool) -> str:
    import carb.settings
    import omni.timeline
    import omni.usd
    from pxr import Gf, UsdGeom, UsdLux

    settings = carb.settings.get_settings()
    settings.set("/persistent/app/viewport/displayOptions", 0)

    context = omni.usd.get_context()
    if not context.open_stage(stage_path):
        raise RuntimeError(f"Could not open stage: {stage_path}")

    stage = context.get_stage()
    if stage is None:
        raise RuntimeError(f"Stage did not load: {stage_path}")

    bbox_cache = UsdGeom.BBoxCache(
        UsdGeom.GetStageMetersPerUnit(stage),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
    )
    default_prim = stage.GetDefaultPrim()
    root_prim = default_prim if default_prim else stage.GetPseudoRoot()
    bbox = bbox_cache.ComputeWorldBound(root_prim).ComputeAlignedBox()
    if bbox.IsEmpty():
        center = Gf.Vec3d(0.0, 0.0, 0.5)
        radius = 2.0
    else:
        center = bbox.GetMidpoint()
        size = bbox.GetSize()
        radius = max(float(size[0]), float(size[1]), float(size[2]), 1.0)

    light = UsdLux.DistantLight.Define(stage, "/World/DocDemoKeyLight")
    light.CreateIntensityAttr(900.0)
    UsdGeom.XformCommonAPI(light).SetRotate((50.0, 0.0, 35.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)

    camera = UsdGeom.Camera.Define(stage, "/World/DocDemoCamera")
    camera.CreateFocalLengthAttr(24.0)
    eye = (
        float(center[0]) + radius * 1.8,
        float(center[1]) - radius * 2.2,
        float(center[2]) + radius * 1.4,
    )
    target = (float(center[0]), float(center[1]), float(center[2]))
    camera.AddTransformOp().Set(look_at_transform(eye=eye, target=target, up=(0.0, 0.0, 1.0)))

    from omni.kit.viewport.utility import get_active_viewport

    viewport = get_active_viewport()
    if viewport:
        viewport.camera_path = "/World/DocDemoCamera"
        viewport.resolution = (width, height)

    if play:
        omni.timeline.get_timeline_interface().play()

    return "/World/DocDemoCamera"


def write_replicator_png(
    app,
    camera_path: str,
    output: Path,
    width: int,
    height: int,
    settle_frames: int,
    capture_frames: int,
) -> None:
    import numpy as np
    import omni.replicator.core as rep
    from PIL import Image

    rep.orchestrator.set_capture_on_play(False)
    render_product = rep.create.render_product(camera_path, (width, height), name="workspace_smoke")
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb.attach(render_product)

    for _ in range(settle_frames):
        app.update()

    print(f"ISAAC_VISUAL_CAPTURE_START output={output}")
    sys.stdout.flush()

    frame = None
    for _ in range(capture_frames):
        rep.orchestrator.step(rt_subframes=4, delta_time=0.0, pause_timeline=False)
        rep.orchestrator.wait_until_complete()
        app.update()
        data = rgb.get_data()
        if isinstance(data, dict):
            data = data.get("data")
        if data is None:
            continue
        array = np.asarray(data)
        if array.size and array.ndim >= 3:
            frame = array[:, :, :3]
            break

    rgb.detach()
    if frame is None:
        raise RuntimeError("Replicator RGB annotator did not return image data.")

    if frame.dtype != np.uint8:
        frame = np.clip(frame, 0, 255).astype(np.uint8)
    Image.fromarray(frame).save(output)
    print("ISAAC_VISUAL_CAPTURE_DONE")
    sys.stdout.flush()


def validate_png(path: Path) -> tuple[int, int, int]:
    if not path.is_file():
        raise RuntimeError(f"Capture did not create {path}")
    if path.stat().st_size < 1024:
        raise RuntimeError(f"Capture is unexpectedly small: {path.stat().st_size} bytes")

    try:
        from PIL import Image, ImageStat
    except Exception:
        return (0, 0, path.stat().st_size)

    with Image.open(path) as image:
        image.load()
        extrema = ImageStat.Stat(image.convert("RGB")).extrema
        if not any(high > low for low, high in extrema):
            raise RuntimeError("Capture image appears to be a flat color.")
        return (image.width, image.height, path.stat().st_size)


def main() -> int:
    args = parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)

    from isaacsim import SimulationApp

    app = SimulationApp(
        {
            "headless": True,
            "width": args.width,
            "height": args.height,
            "limit_cpu_threads": args.cpu_threads,
        }
    )

    try:
        if args.stage:
            camera_path = open_stage_scene(args.stage, args.width, args.height, args.play)
        else:
            camera_path = create_scene(args.width, args.height)
        write_replicator_png(
            app,
            camera_path,
            args.output,
            args.width,
            args.height,
            args.settle_frames,
            args.capture_frames,
        )

        width, height, size = validate_png(args.output)
        print(f"ISAAC_VISUAL_OK label={args.label} output={args.output} size={size} width={width} height={height}")
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)
    except Exception as error:
        print(f"ISAAC_VISUAL_FAILED label={args.label}: {error}", file=sys.stderr)
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(1)


if __name__ == "__main__":
    raise SystemExit(main())
