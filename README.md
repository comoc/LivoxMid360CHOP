# LivoxMid360CHOP

TouchDesigner C++ CHOP that streams live point clouds from a Livox Mid-360 LiDAR into TouchDesigner without any intermediary applications. The operator is built by combining Livox's official [Livox-SDK2](https://github.com/Livox-SDK/Livox-SDK2) networking pipeline with the TouchDesigner plug-in patterns used in [SlamtecCHOP](https://github.com/Ajasra/SlamtecCHOP).

The node exposes Cartesian and derived spherical coordinates plus per-point intensity so that Mid-360 scans can be visualised or further processed inside TouchDesigner.

## Features

- Livox SDK2 (v1.2.x) integration with asynchronous point cloud callbacks.
- Automatic device discovery via the Mid-360 configuration JSON (identical format to Livox samples).
- Configurable point limit per cook and ring-buffer size to handle high-density frames without blocking the TouchDesigner cook thread.
- Live switch between high and low resolution Livox packet formats.
- Output coordinates in Cartesian (XYZ) or spherical (distance/theta/phi) space while keeping raw intensity data.
- Info CHOP/DAT channels that expose connection state, serial/IP address, buffer depth and diagnostic push messages from the device.

## Repository Layout

```
LivoxMid360CHOP.sln                 Visual Studio 2022 solution.
LivoxMid360CHOP.vcxproj            x64 DLL project for the CHOP.
LivoxMid360CHOP.cpp/.h             TouchDesigner CHOP implementation.
LivoxDevice.cpp/.h                 Thin Livox SDK2 wrapper that owns the SDK lifecycle.
Parameters.cpp/.h                  TouchDesigner parameter definitions.
config/mid360_sample.json          Template Mid-360 network configuration.
CHOP_CPlusPlusBase.h, ...          Headers from the TouchDesigner C++ CHOP SDK.
```

## Prerequisites

1. **TouchDesigner C++ SDK headers** – already included (`CHOP_CPlusPlusBase.h`, etc.).
2. **Visual Studio 2022** with the Desktop C++ workload.
3. **Livox-SDK2** cloned alongside this repository (`../Livox-SDK2`) and compiled so that `build/sdk_core/Debug|Release/livox_lidar_sdk_static.lib` exist.
4. Windows x64 environment (TouchDesigner is 64-bit only).
5. A reachable Livox Mid-360 with correct IP/network parameters.

## Building the CHOP

1. Build the Livox SDK static library:
   ```powershell
   cd ../Livox-SDK2
   cmake -B build -S . -A x64
   cmake --build build --config Release
   ```
2. Open `LivoxMid360CHOP.sln` in Visual Studio 2022.
3. Select `x64` and the desired configuration (Debug or Release).
4. Build the solution. The resulting `LivoxMid360CHOP.dll` is generated under `build/<config>/`.
5. Copy the DLL to `Documents/Derivative/TouchDesigner/Plugins/` (or your preferred plug-in search path) and restart TouchDesigner.

> If your Livox SDK folder lives somewhere else, update the `AdditionalIncludeDirectories` and `AdditionalLibraryDirectories` entries in `LivoxMid360CHOP.vcxproj` accordingly.

## TouchDesigner Parameters

| Page | Parameter | Description |
| ---- | --------- | ----------- |
| Connection | `Active` | Enables or stops the SDK instance. |
| Connection | `Config File` | Path to the Mid-360 JSON configuration (see `config/mid360_sample.json`). |
| Streaming | `Points Per Cook` | Number of latest points copied to the CHOP output on each cook. |
| Streaming | `Buffer Limit` | Maximum number of samples cached internally before dropping the oldest ones. |
| Streaming | `Reset Buffer` | Clears the point cache without disconnecting. |
| Output | `Point Data Type` | Request high (millimeter) or low (centimeter) Cartesian packet formats from the lidar. |
| Output | `Coordinate Output` | Choose Cartesian (XYZ) or derived spherical (distance/theta/phi) outputs for the first three channels. Channel 4 always holds intensity. |

The CHOP produces four channels:

1. `x` / `distance`
2. `y` / `theta` (degrees)
3. `z` / `phi` (degrees)
4. `intensity`

Each cook fetches up to `Points Per Cook` samples from the buffered queue. The Info CHOP reports execution count, buffered points, and fill ratio (how many of the requested samples were available). The Info DAT lists the connection status, serial number, lidar IP, totals, and the last diagnostic message broadcast by the device.

## Configuring Livox Mid-360

The JSON matches Livox's own samples. The plugin simply forwards the file path to `LivoxLidarSdkInit`. Use `config/mid360_sample.json` as a starting point and ensure the host IP/ports align with your TouchDesigner machine.

```json
{
  "MID360": {
    "lidar_net_info": { ... },
    "host_net_info": [
      {
        "host_ip": "192.168.1.5",
        "point_data_port": 56301
      }
    ]
  }
}
```

Save the file somewhere accessible (local drive is recommended) and point the `Config File` parameter to it.

## Runtime Notes

- The SDK is initialised only when `Active` is toggled on. The operator is fully idle otherwise.
- Changing the config path re-initialises the SDK so you can switch between different network setups without restarting TouchDesigner.
- Buffer size should exceed `Points Per Cook` to absorb bursts from the sensor. The operator drops the oldest points once the queue limit is exceeded.
- Switching point data format (High/Low) sends `SetLivoxLidarPclDataType` to the device immediately.
- The implementation currently focuses on point clouds. Livox IMU data hooks are in place but not exposed by this CHOP.

## Credits

- Livox SDK team for the Mid-360 networking stack.
- @Ajasra for releasing SlamtecCHOP, which informed the TouchDesigner-specific boilerplate and parameter UX.
