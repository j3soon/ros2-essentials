"""Regression checks for selecting and guarding Go2 X11 proof windows."""

from __future__ import annotations

import os
import subprocess
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import go2_rough_smoke as go2
import proof_capture as proof


class KitWindowTests(unittest.TestCase):
    def test_window_manager_decorations_are_not_duplicate_apps(self):
        with (
            patch.object(go2.shutil, "which", return_value="available"),
            patch.object(
                go2.subprocess,
                "run",
                side_effect=[
                    subprocess.CompletedProcess([], 0, stdout="100\n200\n"),
                    subprocess.CompletedProcess(
                        [], 0, stdout="_NET_CLIENT_LIST(WINDOW): window id # 0xc8\n"
                    ),
                ],
            ),
        ):
            self.assertEqual(go2.visible_kit_windows({"DISPLAY": ":99"}), {"200"})

    def test_missing_xdotool_fails(self):
        with patch.object(go2.shutil, "which", return_value=None):
            with self.assertRaisesRegex(RuntimeError, "requires xdotool"):
                go2.visible_kit_windows({"DISPLAY": ":99"})

    def test_stale_window_cannot_be_used_as_playback_proof(self):
        with (
            patch.object(go2, "visible_kit_windows", return_value={"100"}),
            patch.object(go2.time, "monotonic", side_effect=[0, 31]),
        ):
            with self.assertRaisesRegex(RuntimeError, "visible Isaac Lab window"):
                go2.prepare_kit_capture(
                    {"100"}, env={"DISPLAY": ":99"}, x11_size="1280x720"
                )

    def test_ambiguous_playback_windows_fail(self):
        with patch.object(go2, "visible_kit_windows", return_value={"100", "200"}):
            with self.assertRaisesRegex(RuntimeError, "ambiguous"):
                go2.prepare_kit_capture(
                    set(), env={"DISPLAY": ":99"}, x11_size="1280x720"
                )

    def test_capture_is_bounded_by_the_new_window(self):
        with (
            patch.object(go2, "visible_kit_windows", return_value={"100", "200"}),
            patch.object(go2.subprocess, "run"),
            patch.object(go2, "require_x11_window_visible"),
            patch.object(
                go2.subprocess,
                "check_output",
                return_value="WINDOW=200\nX=30\nY=40\nWIDTH=1001\nHEIGHT=701\nSCREEN=0\n",
            ),
        ):
            self.assertEqual(
                go2.prepare_kit_capture(
                    {"100"}, env={"DISPLAY": ":99"}, x11_size="1280x720"
                ),
                ("200", "1000x700"),
            )


class CaptureGuardTests(unittest.TestCase):
    def test_hidden_or_covered_window_is_rejected(self):
        for visible, active in [("200\n", "100\n"), ("100\n200\n", "200\n")]:
            with (
                self.subTest(visible=visible, active=active),
                patch.object(
                    proof.subprocess,
                    "run",
                    side_effect=[
                        subprocess.CompletedProcess([], 0, stdout=visible),
                        subprocess.CompletedProcess([], 0, stdout=active),
                    ],
                ),
            ):
                with self.assertRaisesRegex(RuntimeError, "not visible"):
                    proof.require_x11_window_visible("100", display=":99")

    def test_screenshot_does_not_capture_a_hidden_window(self):
        with (
            patch.object(proof, "wake_x11_display"),
            patch.object(
                proof, "require_x11_window_visible", side_effect=RuntimeError("hidden")
            ),
            patch.object(proof.subprocess, "run") as capture,
        ):
            with self.assertRaisesRegex(RuntimeError, "hidden"):
                proof.capture_x11_screenshot(
                    Path(__file__).parent / "artifacts" / "hidden.png",
                    display=":99",
                    x11_size="1280x720",
                    window_id="100",
                )
            capture.assert_not_called()

    def test_recording_aborts_and_stops_ffmpeg_when_window_is_covered(self):
        process = Mock()
        process.wait.side_effect = [subprocess.TimeoutExpired("ffmpeg", 0.5), 0]
        process.poll.return_value = None
        with (
            patch.object(proof, "wake_x11_display"),
            patch.object(
                proof,
                "require_x11_window_visible",
                side_effect=[None, None, RuntimeError("covered")],
            ),
            patch.object(proof.subprocess, "Popen") as launch,
        ):
            launch.return_value.__enter__.return_value = process
            with self.assertRaisesRegex(RuntimeError, "covered"):
                proof.record_x11_video(
                    Path(__file__).parent / "artifacts" / "covered.mp4",
                    display=":99",
                    x11_size="1280x720",
                    seconds=10,
                    framerate=15,
                    window_id="100",
                )
            process.terminate.assert_called_once()


@unittest.skipUnless(os.environ.get("WORKSPACE_SMOKE_X11_DISPLAY"), "opt-in live X11")
class LiveX11CaptureTests(unittest.TestCase):
    def test_window_capture_and_occlusion(self):
        import tkinter as tk

        display = os.environ["WORKSPACE_SMOKE_X11_DISPLAY"]
        env = {**os.environ, "DISPLAY": display}
        existing = go2.visible_kit_windows(env)
        root = tk.Tk(screenName=display)
        root.title("Isaac Lab capture regression")
        root.geometry("800x600+10+10")
        canvas = tk.Canvas(root, background="#184c70", highlightthickness=0)
        canvas.pack(fill="both", expand=True)
        for x in range(0, 800, 40):
            canvas.create_line(x, 0, 800 - x, 600, fill="#e0aa50", width=3)
        try:
            root.update()
            window_id, size = go2.prepare_kit_capture(
                existing, env=env, x11_size="1280x720"
            )
            artifacts = Path(__file__).parent / "artifacts" / "review-122"
            artifacts.mkdir(parents=True, exist_ok=True)
            with tempfile.TemporaryDirectory(dir=artifacts) as directory:
                screenshot = Path(directory) / "capture.png"
                self.assertEqual(
                    proof.capture_x11_screenshot(
                        screenshot, display=display, x11_size=size, window_id=window_id
                    ),
                    0,
                )
                recording = Path(directory) / "capture.mp4"
                self.assertEqual(
                    proof.record_x11_video(
                        recording,
                        display=display,
                        x11_size=size,
                        seconds=2,
                        framerate=15,
                        window_id=window_id,
                    ),
                    0,
                )
                self.assertGreater(screenshot.stat().st_size, 0)
                self.assertGreater(recording.stat().st_size, 0)
                cover = tk.Toplevel(root)
                cover.title("Isaac Lab capture obstruction")
                cover.geometry("800x600+10+10")
                root.update()
                cover_id = (go2.visible_kit_windows(env) - existing - {window_id}).pop()
                subprocess.run(
                    ["xdotool", "windowactivate", "--sync", cover_id],
                    env=env,
                    check=True,
                    timeout=10,
                )
                time.sleep(0.2)
                with self.assertRaisesRegex(RuntimeError, "not visible"):
                    proof.capture_x11_screenshot(
                        Path(directory) / "obscured.png",
                        display=display,
                        x11_size=size,
                        window_id=window_id,
                    )
        finally:
            root.destroy()


if __name__ == "__main__":
    unittest.main()
