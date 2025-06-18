#!/bin/bash
set -e

# Install GUI debugging tools and configure Vulkan support

echo "Installing GUI debugging tools..."
# Install GUI debugging tools
# - `x11-apps` and `x11-utils` for `xeyes` and `xdpyinfo`
#   Ref: https://packages.debian.org/sid/x11-apps
#   Ref: https://packages.debian.org/sid/x11-utils
# - `mesa-utils` for `glxgears` and `glxinfo`
#   Ref: https://wiki.debian.org/Mesa
# - `vulkan-tools` for `vkcube` and `vulkaninfo`
#   Ref: https://docs.vulkan.org/tutorial/latest/02_Development_environment.html#_vulkan_packages
#   Ref: https://gitlab.com/nvidia/container-images/vulkan/-/blob/master/docker/Dockerfile.ubuntu
apt-get update && apt-get install -y \
    x11-apps x11-utils \
    mesa-utils \
    libgl1 vulkan-tools \
    && rm -rf /var/lib/apt/lists/*

echo "Configuring Vulkan support..."
# Install Vulkan config files
# Ref: https://gitlab.com/nvidia/container-images/vulkan
# Ref: https://github.com/j3soon/docker-vulkan-runtime
cat > /etc/vulkan/icd.d/nvidia_icd.json <<EOF
{
    "file_format_version" : "1.0.0",
    "ICD": {
        "library_path": "libGLX_nvidia.so.0",
        "api_version" : "1.3.194"
    }
}
EOF

mkdir -p /usr/share/glvnd/egl_vendor.d
cat > /usr/share/glvnd/egl_vendor.d/10_nvidia.json <<EOF
{
    "file_format_version" : "1.0.0",
    "ICD" : {
        "library_path" : "libEGL_nvidia.so.0"
    }
}
EOF

cat > /etc/vulkan/implicit_layer.d/nvidia_layers.json <<EOF
{
    "file_format_version" : "1.0.0",
    "layer": {
        "name": "VK_LAYER_NV_optimus",
        "type": "INSTANCE",
        "library_path": "libGLX_nvidia.so.0",
        "api_version" : "1.3.194",
        "implementation_version" : "1",
        "description" : "NVIDIA Optimus layer",
        "functions": {
            "vkGetInstanceProcAddr": "vk_optimusGetInstanceProcAddr",
            "vkGetDeviceProcAddr": "vk_optimusGetDeviceProcAddr"
        },
        "enable_environment": {
            "__NV_PRIME_RENDER_OFFLOAD": "1"
        },
        "disable_environment": {
            "DISABLE_LAYER_NV_OPTIMUS_1": ""
        }
    }
}
EOF

echo "GUI debugging tools and Vulkan configuration completed successfully!"
