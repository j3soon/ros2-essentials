#!/usr/bin/env python3
"""Create a visible Isaac Sim GUI proof scene and keep it open for capture."""

from __future__ import annotations

import argparse
import signal
import time

from isaacsim import SimulationApp


stop_requested = False


def request_stop(_signum: int, _frame: object) -> None:
    global stop_requested
    stop_requested = True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--settle-frames", type=int, default=60)
    parser.add_argument(
        "--hold-seconds",
        type=float,
        default=300.0,
        help="Seconds to keep the GUI open after readiness; use 0 or less to hold until stopped.",
    )
    parser.add_argument("--cpu-threads", type=int, default=16)
    return parser.parse_args()


def author_scene(width: int, height: int) -> None:
    import omni.usd
    from isaacsim.core.rendering_manager import ViewportManager
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade

    context = omni.usd.get_context()
    context.new_stage()
    stage = context.get_stage()

    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())

    cube = UsdGeom.Cube.Define(stage, "/World/ProofCube")
    cube.CreateSizeAttr(1.0)
    UsdGeom.XformCommonAPI(cube).SetTranslate((0.0, 0.0, 0.55))
    cube.GetPrim().CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set(
        [Gf.Vec3f(0.05, 0.38, 0.95)]
    )

    material = UsdShade.Material.Define(stage, "/World/ProofBlue")
    shader = UsdShade.Shader.Define(stage, "/World/ProofBlue/PreviewSurface")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.05, 0.38, 0.95))
    shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.01, 0.04, 0.12))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.35)
    material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    UsdShade.MaterialBindingAPI(cube.GetPrim()).Bind(material)

    ground = UsdGeom.Cube.Define(stage, "/World/Ground")
    ground.CreateSizeAttr(1.0)
    UsdGeom.XformCommonAPI(ground).SetScale((4.0, 4.0, 0.025))
    UsdGeom.XformCommonAPI(ground).SetTranslate((0.0, 0.0, -0.025))
    ground.GetPrim().CreateAttribute("primvars:displayColor", Sdf.ValueTypeNames.Color3fArray).Set(
        [Gf.Vec3f(0.72, 0.72, 0.66)]
    )

    light = UsdLux.DistantLight.Define(stage, "/World/KeyLight")
    light.CreateIntensityAttr(900.0)
    UsdGeom.XformCommonAPI(light).SetRotate((45.0, 0.0, 35.0), UsdGeom.XformCommonAPI.RotationOrderXYZ)

    dome = UsdLux.DomeLight.Define(stage, "/World/DomeLight")
    dome.CreateIntensityAttr(250.0)

    ViewportManager.set_camera_view(
        "/OmniverseKit_Persp",
        eye=[3.0, -4.0, 2.4],
        target=[0.0, 0.0, 0.45],
    )

    print(f"ISAAC_GUI_SCENE_AUTHORED path=/World/ProofCube resolution={width}x{height}", flush=True)


def main() -> int:
    signal.signal(signal.SIGTERM, request_stop)
    signal.signal(signal.SIGINT, request_stop)
    args = parse_args()

    app = SimulationApp(
        {
            "headless": False,
            "width": args.width,
            "height": args.height,
            "limit_cpu_threads": args.cpu_threads,
        }
    )

    try:
        author_scene(args.width, args.height)

        for _ in range(max(args.settle_frames, 0)):
            if stop_requested:
                print("ISAAC_GUI_SCENE_STOPPED before_ready=true", flush=True)
                return 130
            app.update()

        print(f"ISAAC_GUI_SCENE_READY path=/World/ProofCube resolution={args.width}x{args.height}", flush=True)

        deadline = None if args.hold_seconds <= 0 else time.monotonic() + args.hold_seconds
        while not stop_requested and (deadline is None or time.monotonic() < deadline):
            app.update()
            time.sleep(1.0 / 30.0)
    finally:
        app.close()

    print("ISAAC_GUI_SCENE_CLOSED", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
