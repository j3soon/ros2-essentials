# NVIDIA OpenUSD Tools

[![GitHub code](https://img.shields.io/badge/code-blue?logo=github&label=github)](https://github.com/j3soon/ros2-essentials/blob/main/docker_modules/install_nv_openusd.sh)

[NVIDIA OpenUSD binary tools](https://developer.nvidia.com/openusd#section-getting-started) v25.08 for Linux.

To enable NVIDIA OpenUSD tools, set the `NV_OPENUSD` argument to `YES` in the `compose.yaml` file of your desired workspace (e.g., `template_ws/docker/compose.yaml`). After making these changes, rebuild the Docker image.

## Usage

These OpenUSD tools are often very useful for working with USD files directly without needing to launch Isaac Sim.

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

> **Limitations**: Certain USD tools (e.g., `usdrecord` and `usdchecker`) may still have runtime issues when executing directly due to upstream missing dependencies or compatibility problems. See the following for workarounds:
>
> ```
> FILE=~/nvidia/openusd/share/usd/tutorials/traversingStage/HelloWorld.usda
> PYTHONHOME=${HOME}/nvidia/openusd/python \
> PYTHONPATH= \
> USD_INSTALL_DIR=${HOME}/nvidia/openusd \
> QT_QPA_PLATFORM=offscreen \
> usdrecord.sh "$FILE" output.png
> ```
>
> ```
> FILE=~/nvidia/openusd/share/usd/tutorials/traversingStage/HelloWorld.usda
> PYTHONHOME=${HOME}/nvidia/openusd/python \
> PYTHONPATH= \
> USD_INSTALL_DIR=${HOME}/nvidia/openusd \
> usdchecker.sh $FILE
> ```

## Comparison with Official Pixar OpenUSD

The NVIDIA pre-built OpenUSD tools are used here since they provide pre-built binaries. The official Pixar OpenUSD doens't provide pre-built binaries, you'll need to build from source if you want to use it by referring to the [Pixar USD documentation](https://github.com/PixarAnimationStudios/OpenUSD) for instructions.

## References

- [NVIDIA OpenUSD binary tools](https://developer.nvidia.com/openusd#section-getting-started)
- [usdview](https://docs.omniverse.nvidia.com/usd/latest/usdview/quickstart.html):
- [usdview - Omniverse USD](https://docs.omniverse.nvidia.com/usd/latest/usdview/quickstart.html):
- [USD Toolset - OpenUSD](https://openusd.org/release/toolset.html)
