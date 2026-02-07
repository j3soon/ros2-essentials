# NVIDIA OpenUSD Tools

[![GitHub code](https://img.shields.io/badge/code-blue?logo=github&label=github)](https://github.com/j3soon/ros2-essentials/blob/main/docker_modules/install_nv_openusd.sh)

[NVIDIA OpenUSD binary tools](https://developer.nvidia.com/openusd#section-getting-started) v25.08 for Linux.

To enable NVIDIA OpenUSD tools, set the `NV_OPENUSD` argument to `YES` in the `compose.yaml` file of your desired workspace (e.g., `template_ws/docker/compose.yaml`). After making these changes, rebuild the Docker image.

## Usage

```sh
FILE=~/nvidia/openusd/share/usd/tutorials/traversingStage/HelloWorld.usda
FILE_REF=~/nvidia/openusd/share/usd/tutorials/traversingStage/RefExample.usda
usdview_gui.sh
usdedit.sh $FILE
usdcat.sh $FILE
usddiff.sh $FILE $FILE_REF
usdview.sh $FILE
# usdrecord.sh $FILE output.png
usdresolve.sh $FILE
usdtree.sh $FILE
usdtree.sh -f $FILE_REF
usdzip.sh output.usdz $FILE
# usdchecker.sh $FILE
# and more ...
```

Check `~/nvidia/openusd/scripts` and [USD Toolset documentation](https://openusd.org/release/toolset.html) for the full list of available tools and their usage.

> **Limitations**: Certain USD tools (e.g., `usdrecord` and `usdchecker`) may still have runtime issues due to upstream missing dependencies or compatibility problems.

## References

- [NVIDIA OpenUSD binary tools](https://developer.nvidia.com/openusd#section-getting-started)
- [usdview](https://docs.omniverse.nvidia.com/usd/latest/usdview/quickstart.html):
- [usdview - Omniverse USD](https://docs.omniverse.nvidia.com/usd/latest/usdview/quickstart.html):
- [USD Toolset - OpenUSD](https://openusd.org/release/toolset.html)
