---
name: realsense-capture
description: Capture live RealSense RGB and depth PNGs and short videos using the existing template_ws container, without installing RealSense dependencies on the host. Use for direct camera stream capture and preservation of 16-bit depth measurements.
---

# RealSense Capture

Use the working `template_ws` container for camera access, `pyrealsense2`,
NumPy, and OpenCV. Keep RealSense dependencies inside the container. Do not
install host SDK packages, Python bindings, kernel modules, or udev rules.
An existing host FFmpeg and FFprobe can encode and validate saved frames when
the container does not provide those executables.

## Container and Device Access

Check the current container state and reuse the local image. If
`ros2-template-ws` exists but is stopped, start it with `docker start`. If it
does not exist, use Compose without rebuilding or pulling:

```bash
USER_UID=$(id -u) docker compose -f template_ws/docker/compose.yaml \
  up -d --no-build --pull never
```

The workspace Compose configuration supplies privileged USB access and the
repository mount at `/home/ros2-essentials`. Check the actual mounts when using
an existing container. See [the module documentation](../../docs/docker-modules/realsense.md)
for the workspace setup.

Use the installed SDK directly, without sourcing a ROS shell:

```bash
docker exec --user root ros2-template-ws /usr/local/bin/rs-enumerate-devices -s
docker exec --user root ros2-template-ws python3 -c \
  'import pyrealsense2, cv2, numpy; print(pyrealsense2.__file__)'
```

A ROS-provided `rs-enumerate-devices` may detect the camera even when the
RSUSB-backed Python SDK cannot open it. In the tested setup, the default
container user encountered `failed to set power state` with
`RS2_USB_STATUS_ACCESS`. Running the capture as root inside the container
resolved USB permissions without host changes. Other power-state errors need
their own diagnosis. Stop if another process owns the camera rather than
terminating an unrelated viewer or driver.

Enumerate devices and supported profiles. Select the connected serial at
runtime. Ask which camera to use if multiple devices are available. The tested
D455 supports the template settings of RGB BGR8 and depth Z16 at 848 by 480,
30 fps. Check [the current configuration](../../template_ws/src/realsense_examples/realsense_launch/config/realsense.yaml)
instead of assuming those settings apply to every device.

## Paired Capture

Write session files under a fresh directory in
`tests/workspace_smoke/artifacts/realsense/`. Store any reusable capture helper
under `tests/workspace_smoke/modules/realsense/`, outside the ignored artifact
directory. Run camera code with `docker exec --user root`. Add `-i` when passing
Python code through standard input.

Configure one SDK pipeline with both color and depth enabled. Allow exposure
to settle before saving frames. Three seconds of warmup worked for the tested
D455. Acquire paired frames with `pipeline.wait_for_frames()`, copy each array
before releasing its SDK frame, and stop the pipeline in `finally`.

For five seconds at 30 fps, collect 150 unique color/depth pairs. Record frame
numbers, sensor timestamps, timestamp domains, and arrival times. Check that
both streams advance. Record gaps or duplicates and compare sensor elapsed
time with the requested duration. Encoding 150 frames at 30 fps proves a
five-second video, but skipped frames can make the actual capture interval
longer. Use a timeout so acquisition cannot wait indefinitely.

Use `rs.align(rs.stream.color)` when aligned RGB and depth are wanted. Keep
the native depth frames as well when preserving the original sensor geometry
matters. Save the first recorded pair as the PNG snapshots:

- `rgb.png`: BGR8 input saved through OpenCV, which writes correct PNG colors.
- `depth.png`: aligned `uint16` depth, saved without normalization.
- `depth_preview.png`: colorized aligned depth for viewing.
- Optional `depth_native.png`: original `uint16` depth before alignment.

Record `get_depth_scale()`. Distance in meters is the pixel value multiplied
by that scale, and zero denotes invalid depth. The tested D455 used about
0.001 meters per unit. Store color and native depth intrinsics, depth-to-color
extrinsics, profile settings, and UTC capture times in `metadata.json`.

Use a fixed range for the depth preview, such as 0 to 4 meters, with a named
colormap and invalid pixels black. Record the range and colormap. Frame-wise
normalization makes colors change even when measured distances stay constant.
Keep preview pixels separate from the quantitative 16-bit depth data.

## Encoding

Capture arrays in the container, then encode from saved raw frames. At the
tested settings, write color and preview frames as packed `bgr24` and depth
frames as little-endian `gray16le`. Buffering the short recording before
encoding avoids encoder delays during acquisition. Preserve raw files until
validation succeeds. Use an unused output directory so retries cannot
overwrite earlier captures.

These commands assume the working directory contains 150 captured frames at
848 by 480. Substitute the actual dimensions, frame count, and frame rate:

```bash
ffmpeg -hide_banner -loglevel error -nostdin \
  -f rawvideo -pixel_format bgr24 -video_size 848x480 -framerate 30 \
  -i rgb.bgr -frames:v 150 -an -c:v libx264 -crf 18 \
  -pix_fmt yuv420p -movflags +faststart rgb.mp4

ffmpeg -hide_banner -loglevel error -nostdin \
  -f rawvideo -pixel_format bgr24 -video_size 848x480 -framerate 30 \
  -i depth_preview.bgr -frames:v 150 -an -c:v libx264 -crf 18 \
  -pix_fmt yuv420p -movflags +faststart depth_preview.mp4

ffmpeg -hide_banner -loglevel error -nostdin \
  -f rawvideo -pixel_format gray16le -video_size 848x480 -framerate 30 \
  -i depth.u16 -frames:v 150 -an -c:v ffv1 -level 3 \
  -pix_fmt gray16le depth.mkv
```

H.264 MP4 is suitable for RGB and the depth preview. Preserve quantitative
depth in a lossless 16-bit FFV1 MKV. OpenCV 4.5.4's usual VideoWriter path does
not preserve these 16-bit input frames.

## Validation and Cleanup

Use FFprobe to check dimensions, pixel format, decoded frame count, and
duration. Decode every video with FFmpeg to catch corrupt streams. Decode each
FFV1 video to `gray16le` and compare its SHA256 with the source raw depth bytes.
Check PNG shape and dtype through container OpenCV, and compare each PNG with
its first captured frame.

Inspect the RGB PNG, colorized depth PNG, and a frame extracted from each
preview video. A very dark RGB view can coexist with valid depth. Report that
observation and preserve the captured view rather than changing camera
settings or brightening the requested output without instruction.

Write validation results and artifact paths alongside the capture metadata.
Remove temporary raw files only after verification. Return root-created files
to the host user's actual UID and GID. Stop a container started for this task
when finished, while preserving any container that was already running. Link
the requested PNGs and videos in the final response, distinguish the depth
preview from raw depth, and confirm that no host dependencies were installed.
