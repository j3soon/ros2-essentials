# ZED Camera extension

The ZED camera extension allow you to stream virtual ZED camera data to the ZED SDK.

## Getting started

### Adding the 3D model of the ZED

Navigate to this extension's folder in the content browser, you can drag and drop the `ZED_X.usd` model under data directly into your stage.

### Starting the data stream

To enable the streaming in Isaac sim, add the ZED Camera Omnigraph node to an Action graph, connect your camera to it, configure the parameters, and press play.

#### Parameters

You can configure the streamer properties in the Property panel on the right:
- `Serial number`- this field allows you identify the virtual camera you are using. It has no functional effect and can be left to the default value. This field can be useful when using multiple camera and you need to identify each one.
- `Streaming port` - Defines which port is used for streaming. It must be an even number.
- `Use system time` - when set to true, the streamer assigns the latest system time to the images that are sent to the SDK. If set to false, the streamer only sets the first timestamp to system time and then increments the later timestamps by simulation time.
- `ZED Camera prim` - this parameter should be set to the ZED X camera prim in the stage as shown in the picture above.

### Connecting to the ZED SDK

Once you press play in the simulation, you can connect the virtual camera to the ZED SDK via the streaming interface.

## Documentation

Please refer to the [StereoLabs' website for the full documentation](https://www.stereolabs.com/docs/isaac-sim/setting_up_zed_isaac_sim).


