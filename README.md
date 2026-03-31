[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/humble/)
[![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-blue?logo=ros&logoColor=white)](https://docs.ros.org/en/jazzy/)
[![C++17](https://img.shields.io/badge/C%2B%2B-14-00599C?logo=cplusplus&logoColor=white)](https://en.cppreference.com/w/cpp/14)
[![CUDA](https://img.shields.io/badge/CUDA-Optional-76B900?logo=nvidia&logoColor=white)](https://developer.nvidia.com/cuda-toolkit)
[![CMake](https://img.shields.io/badge/CMake-3.5+-064F8C?logo=cmake&logoColor=white)](https://cmake.org/)
[![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?logo=opencv&logoColor=white)](https://opencv.org/)
[![Vimba SDK](https://img.shields.io/badge/Vimba-SDK-informational)](https://www.alliedvision.com/en/products/vimba-sdk/)
[![image_transport](https://img.shields.io/badge/image__transport-ROS2-blue)](http://wiki.ros.org/image_transport)
[![libpcap](https://img.shields.io/badge/libpcap-PCAP%20replay-informational)](https://www.tcpdump.org/)
[![License](https://img.shields.io/badge/License-BSD-green.svg)]()

# avt_vimba_camera (ROS2)

This repo contains a ROS2 driver for cameras manufactured by [Allied Vision Technologies](https://www.alliedvision.com).
The driver relies on libraries provided by AVT as part of their [Vimba SDK](https://www.alliedvision.com/en/products/software.html).

*See the ROS1 version of this README [here](https://github.com/astuff/avt_vimba_camera/blob/ros1_master/README.md).*

## Installation

### Dependencies
First, you will need to install the Vimba SDK.
Download it from AVT's website [here](https://www.alliedvision.com/en/products/vimba-sdk/#c1497).

Also see the [linux vimba installation instructions](https://cdn.alliedvision.com/fileadmin/content/documents/products/software/software/Vimba/appnote/Vimba_installation_under_Linux.pdf).

It is highly recommended to open the "Vimba Viewer" tool that came along with the SDK and make sure you can connect to your camera.

It may be useful to create a desktop shortcut to Vimba Viewer:
```sh
ln -sf "Vimba_5_0/Tools/Viewer/Bin/x86_64bit/VimbaViewer" "$HOME/Desktop"
```

### ROS Driver

Once you've successfully connected to your camera using Vimba Viewer, you can continue with the ROS driver install:

```
sudo apt install ros-$ROS_DISTRO-avt-vimba-camera
```

## Operational Advice

### MTU Size (GigE Cameras)
If you are using a GigE camera (ethernet-based camera), it is recommended to adjust some settings in your network interface to be able to handle the potentially high bandwidth usage of the camera stream.

On Linux, you will need to increase the MTU (Maximum Transmission Unit) on the network interface attached to the camera.

You can check what your current mtu setting is by running the following command:
```
ip a | grep mtu
``` 

According to AVT documentation, increase the mtu to `9014`.
If you use Network Manager, this can be done by opening the network interface settings and editing the "MTU" box under the "Identity" tab. 

See the "Optimize system performance" section of your camera's technical manual for full details.
For example, the Mako camera technical manual is available [here](https://cdn.alliedvision.com/fileadmin/content/documents/products/cameras/Mako/techman/Mako_TechMan_en.pdf).

### Receive Buffer Size (GigE Cameras)

It is also recommended to increase your network receive buffer size.
By default, Ubuntu uses `212992`.

You can check what your current buffer size is:
```
sudo sysctl 'net.core.rmem_max'
```
Update the buffer size with the following command:
```
sudo sysctl -w 'net.core.rmem_max=26214400'
```

`26214400` has been tested successfully, but anything above `2000000` is likely fine.

Once you find a value that works for you, you can make the change permanent (persist across reboots) by updating the `/etc/sysctl.conf` file with the following line:

```
net.core.rmem_max=26214400
```

### Camera Settings in General

When the ROS2 driver starts up, it queries the camera for all features and creates ROS parameters for every feature on the camera.
All camera-related ROS parameters are prefixed with "feature/" in their name to indicate they are camera features.
Note that some features are read-only, and each camera model will have slightly different feature sets.
If the user does not specify the ROS parameter for a given feature using yaml config files or launch files, then the value for that feature is untouched and not overridden by the driver.

This allows you to take two different approaches towards params/features with the ROS driver:
1. Don't configure anything via ROS params and instead use the camera's "Saved User Sets" functionality to load a custom configuration every time it boots. 
The ROS driver won't change anything (as long as no "feature/" ROS params are set) and just use the camera as it's configured.

2. Use ROS params defined in a yaml file to configure the camera when the driver starts.
Existing params can be saved using `ros2 param dump [node_name]` .

Reagrdless of your approach, if you are having difficulty getting the camera configured, it is suggested to first use Vimba Viewer to play around with settings that work. 
The Vimba Viewer GUI will help you determine what settings are available to your camera model and help you tune them easier.
Once you have settings that you are happy with, save them into your own rosparam file or onto the camera using "Saved User Sets". 


## ROS Nodes

### mono_camera_node

The mono_camera_node is the main driver that connects to the camera, configures it according to ROS parameters/dynamic reconfigure, and starts publishing image frames.
The driver uses [image_transport](http://wiki.ros.org/image_transport) to publish image frames, so all expected image topics should be available.
See the config file (`cfg/AvtVimbaCamera.cfg`) for documentation regarding the various parameters that can be used to configure the camera itself.
See the launch file (launch/mono_camera.launch) for documentation regarding the operational parameters of the driver.

### trigger_node

The trigger_node is a standalone node for sending out ethernet-based action commands to AVT cameras. 
Action commands are useful for triggering frame captures over ethernet.
See AVT's [application note](https://cdn.alliedvision.com/fileadmin/content/documents/products/cameras/various/appnote/GigE/Action-Commands_Appnote.pdf) for more details.
Note that cameras must be configured to receive the action commands in addition to running the trigger_node.

## Clock Synchronization

If you wish to use the exact time the image was measured in the header of the ROS messages, it is suggested to use PTP synchronization.
PTP will ensure the clock on the camera is synchronized with the computer, so that measurement times are all based off of the same clock.
Setting the `use_measurement_time` parameter will set the ROS header timestamp to the frame timestamp, but it is up to you to make sure the camera clock is synced with the computer.
[linuxptp](http://linuxptp.sourceforge.net) is a great tool for PTP synchronization and is suggested for ensuring the camera is in sync with the computer.
See the links below for more details on PTP sync.

## Useful Technical References and Application Notes

- [GigE Features Reference](https://cdn.alliedvision.com/fileadmin/content/documents/products/cameras/various/features/GigE_Features_Reference.pdf) (To better understand what features your camera supports and how to tune them)
- [Trigger over Ethernet - Action Commands](https://cdn.alliedvision.com/fileadmin/content/documents/products/cameras/various/appnote/GigE/Action-Commands_Appnote.pdf) 
- [PTP Clock Sync](https://cdn.alliedvision.com/fileadmin/content/documents/products/cameras/various/appnote/GigE/PTP_IEEE1588_with_Prosilica_GT_GC_Manta.pdf) (Highly recommended if you care about exact image acquisition time)
- [Image Timestamp on Allied Vision GigE Cameras](https://cdn.alliedvision.com/fileadmin/content/documents/products/cameras/various/appnote/GigE/Image_Timestamp.pdf) 
- [Decimation](https://cdn.alliedvision.com/fileadmin/content/documents/products/cameras/various/appnote/various/Decimation.pdf) (Binning is similar)

## `cuda_camera_node` (v2.0.0-configurable)

> **Hardware validation status (2026-03-27) -- none of the items below have been tested on real Mako G cameras:**
>
> - [ ] `cuda_camera_node` receives frames from a real Mako G camera
> - [ ] Single camera sustains 30+ fps (RGB8 debayer, no ROI)
> - [ ] Single camera sustains 39+ fps (Bayer raw, no ROI)
> - [ ] Single camera sustains 55+ fps (Bayer raw + ROI crop)
> - [ ] 6-camera simultaneous launch -- all topics publishing
> - [ ] PTP timestamp sync -- <1 us delta across cameras
> - [ ] Zero dropped frames over a 10-minute run
> - [ ] PCAP replay produces frames identical to live capture
> - [ ] `gpu_direct` DLPack handshake with downstream JAX perception

Zero-copy CUDA alternative to `mono_camera_node`, designed for low-latency autonomous racing perception on the IAC Dallara IL-15. Opt-in via the `BUILD_CUDA_NODE` CMake option. Same external interface (topics, parameters, services) as `mono_camera_node`.

---

### Why This Exists

The stock `mono_camera_node` (upstream `astuff/avt_vimba_camera`) topped out at ~13 fps per camera with 9.6 MB RGB8 frames. For 6 Mako G cameras at 40 Hz over GigE, this was insufficient:

1. **Bandwidth**: 9.6 MB/frame x 40 Hz x 6 cameras = 2.3 GB/s, far exceeding a single GigE link's ~125 MB/s. Even with 4 NICs, 13 fps per camera was the ceiling.
2. **Redundant debayer**: The driver debayered Bayer to RGB on CPU, then the downstream perception ISP undistorted and resized the RGB image. The ISP can consume raw Bayer directly and do debayer + undistort + resize in one fused GPU kernel -- the driver debayer was wasted work that also tripled message size.
3. **Timestamp jitter**: Without PTP, each camera's timestamp came from the system clock at frame-arrival time. At 170 mph, a 10 ms offset between two cameras = 0.76 m position error in cross-camera fusion.
4. **Buffer aliasing race**: The upstream VmbCPP frame observer re-queued buffers inside the callback, creating a race condition where Vimba could DMA into a buffer that the publisher was still reading. This caused corrupted frames and occasional segfaults under load.

---

### What Changed from `mono_camera_node`

#### v1.0.0: Core rewrite

| Change | Details | Motivation |
|--------|---------|------------|
| **VmbC instead of VmbCPP** | Raw C API (`VmbC`) replaces the C++ wrapper | Lower overhead, no hidden allocations, direct control over buffer lifecycle |
| **Pinned DMA buffers** | `cudaMallocHost` pool of 6 page-locked buffers, registered with Vimba as frame receive targets | Zero-copy DMA from camera NIC to pinned host memory, no intermediate copies |
| **Lock-free SPSC queue** | Frame callback pushes `FrameEntry` structs to a single-producer/single-consumer ring buffer; publisher thread polls at 100 us intervals | Decouples Vimba's internal callback thread from ROS publishing, no mutexes on the hot path |
| **Deferred frame re-queue** | Vimba frame buffers are re-queued after the publisher finishes reading, not inside the callback | Fixes the buffer aliasing race (D1) -- the root cause of corrupted frames in the upstream driver |
| **CUDA Malvar-He-Cutler debayer** | 5x5 high-quality demosaicing kernel with shared-memory tiling (16x16 blocks, 2-pixel halo) | When debayer is needed (debug/compat mode), runs on GPU in ~0.3 ms instead of ~2 ms on CPU |
| **PTP timestamp extraction** | Reads PTP timestamp from Vimba frame metadata and converts to ROS time | Enables sub-microsecond cross-camera sync |
| **Wall-clock fallback** | Uses `std::chrono::system_clock` instead of `steady_clock` when PTP is unavailable | `steady_clock` has no relation to UTC and cannot be compared across processes; fixes the D2 timestamp drift bug |
| **Camera open retry** | Retries `VmbCameraOpen` up to 30 times at 2-second intervals | GigE cameras take several seconds to become addressable after power-on; the upstream driver failed immediately (E5 fix) |
| **Auto packet size** | Calls `VmbFeatureCommandRun("GVSPAdjustPacketSize")` on startup | Auto-negotiates jumbo frames without manual MTU matching (E3 fix) |
| **PCAP replay** | `pcap_reader.cpp` reassembles GigE Vision GVSP packets from `.pcap` captures, feeds them through the same pipeline | Enables offline testing with recorded raw GigE packets without camera hardware |

#### v2.0.0-configurable: Toggleable optimizations

All v1.0.0 changes were hardcoded. v2.0.0 exposes them as ROS parameters:

| Parameter | Type | Default | Effect |
|-----------|------|---------|--------|
| `publish_raw_bayer` | bool | `true` | Skip GPU debayer, publish `bayer_rggb8` (1 byte/px) instead of `rgb8` (3 bytes/px) |
| `roi_enabled` | bool | `false` | On-sensor ROI crop (camera only reads out the specified rectangle) |
| `roi_offset_x` | int | `0` | ROI X offset in pixels |
| `roi_offset_y` | int | `515` | ROI Y offset -- default skips top 1/3 of frame (sky on the Dallara) |
| `roi_width` | int | `0` | 0 = full sensor width (2064) |
| `roi_height` | int | `0` | 0 = auto: sensor_height - offset_y |
| `enable_ptp_sync` | bool | `true` | IEEE 1588 PTP slave mode with 10-second lock polling |
| `gpu_direct` | bool | `false` | Expose pinned/device buffer pointers for zero-copy composed launch |
| `num_buffers` | int | `6` | Number of pinned DMA frame buffers in the pool |
| `ptp_offset` | int | `-37` | Nanosecond TAI-to-UTC correction (launch default) |
| `enable_pcap` | bool | `false` | PCAP replay mode |
| `pcap_file` | string | `""` | Path to `.pcap` capture file |

---

### Architecture

```
Mako G sensor (2064x1544 BayerRG8)
  |
  | GigE Vision (GVSP over UDP, jumbo frames 9014 MTU)
  |
  v
VmbC frame receive callback (Vimba SDK internal thread)
  |-- Extracts PTP timestamp (or falls back to system_clock)
  |-- Pushes FrameEntry to lock-free SPSC queue
  |-- Does NOT re-queue buffer yet (D1 fix)
  v
Publisher thread (dedicated, polls SPSC at 100us)
  |
  |-- If publish_raw_bayer=true:
  |     memcpy pinned -> ROS Image msg (bayer_rggb8, 3.2 MB)     ~0.1ms
  |
  |-- If publish_raw_bayer=false:
  |     cudaMemcpyAsync H2D -> debayer kernel -> cudaMemcpyAsync D2H
  |     -> ROS Image msg (rgb8, 9.6 MB)                           ~0.5ms
  |
  |-- If gpu_direct=true:
  |     Ring of 3 GPU device buffers
  |     get_latest_gpu_frame() / release_gpu_frame() API
  |     Zero-copy to perception via DLPack (composed launch)     ~0.01ms
  |
  v
Re-queue Vimba frame buffer (safe: publisher is done reading)
```

---

### Three Configuration Modes

#### Mode 1: "Safe / Debug" -- `publish_raw_bayer=false, roi_enabled=false, gpu_direct=false`

The driver debayers on GPU and publishes standard RGB8 images. Closest behavior to the upstream `mono_camera_node` but with CUDA debayer instead of CPU. Appropriate for initial bring-up, debugging, rviz viewing, and compatibility with nodes expecting `encoding: rgb8`.

> **WARNING:** Bandwidth-limited to ~13 fps per camera per GigE NIC. At 40 Hz with 6 cameras, a minimum of 4 dedicated NICs is required with zero headroom. Frame drops are likely under load.

**Performance:** 9.6 MB/frame, ~13 fps max per NIC, ~0.5 ms driver latency.

#### Mode 2: "Racing" -- `publish_raw_bayer=true, roi_enabled=true, gpu_direct=false` (recommended)

The driver publishes raw Bayer data without debayering (3x smaller). The on-sensor ROI crop skips the top 1/3 of the frame (sky). The downstream perception ISP handles debayer + undistort + resize in one fused kernel.

> **WARNING:** Raw Bayer output breaks any node expecting `encoding: rgb8`. This includes the IAC_Perception health checks, calibration targets, and aruco detection.

> **WARNING:** ROI crop removes the top 1/3 of the image. On banked turns, steep hills, or overpasses, cars may appear in the cropped region. Side cameras must have ROI disabled (no sky visible) -- use per-camera overrides.

> **WARNING:** rviz and foxglove display green-tinted images for `bayer_rggb8`. Cosmetic only -- the data is correct. Mode 1 is appropriate for visual debugging.

**Performance:** 2.1 MB/frame, ~59 fps max per NIC, ~0.1 ms driver latency.

#### Mode 3: "Zero-Copy" -- `publish_raw_bayer=true, roi_enabled=true, gpu_direct=true`

Same as Mode 2, plus exposes pinned DMA buffer pointers via a shared-memory API for zero-copy frame access in a composed launch (driver + perception in the same process).

> **WARNING:** This is the least tested mode. The DLPack handshake between the C++ driver and Python JAX perception has been designed but never run end-to-end. Mode 2 must be validated on hardware first.

> **WARNING:** Composed launch means driver and perception share a process. A crash in perception takes down the camera driver.

**Performance:** 2.1 MB/frame, ~59 fps max per NIC, ~0.01 ms transfer latency.

---

### Known Issues

| Issue | Severity | Details |
|-------|----------|---------|
| **No hardware test** | CRITICAL | The `cuda_camera_node` has never received a frame from a real Mako G camera. The VmbC API call sequence is based on SDK docs and upstream VmbCPP code, but subtle differences may cause silent failures. |
| **Buffer aliasing (D1)** | Fixed (untested) | The deferred re-queue fix is structurally correct but has not been stress-tested under actual GigE DMA timing. A race could still exist if Vimba's internal timeout re-queues a buffer before the publisher releases it. |
| **PTP lock detection** | Untested | The driver polls `PtpStatus` every 10 seconds. Not verified with any specific PTP grandmaster or switch. The `ptp_offset` default (-37 ns) is a placeholder -- the real TAI-UTC offset depends on the PTP domain. |
| **ROI + multi-camera** | Untested | Per-camera ROI overrides are parsed but not verified through VmbC in a multi-camera launch. |
| **PCAP replay** | Untested | Assumes 30 fps / 33 ms frame intervals and standard GVSP packet layout. No `.pcap` files from these cameras exist to test against. |
| **`load_settings` / `save_settings`** | Stub | Both ROS services return "not yet implemented" warnings. |
| **Watchdog timeout** | Hardcoded 2.0s | May need tuning -- too short triggers spurious kills during PTP lock acquisition, too long delays dead-camera detection. |
| **6-camera launch** | Untested | `all_cameras_cuda.launch.py` launches 6 instances with link-local IPs 169.254.100.1-6. Resource contention across multiple NICs has not been verified. |
| **Dead code in publisher** | Minor | Unreachable duplicate publish code after the if/else for raw_bayer/rgb paths (~lines 1018-1024 in `cuda_camera_node.cpp`). |

---

### Performance (Estimated -- Not Validated)

| Config | Frame Size | Max FPS (1 GigE NIC) | Driver Latency | Source |
|--------|-----------|---------------------|----------------|--------|
| RGB8 full (`mono_camera_node`) | 9.6 MB | ~13 fps | ~8-15 ms (CPU debayer) | Measured on upstream driver |
| RGB8 full (`cuda_camera_node`) | 9.6 MB | ~13 fps | ~0.5 ms (CUDA debayer) | Estimated from kernel timing |
| Bayer raw | 3.2 MB | ~39 fps | ~0.1 ms (memcpy) | Calculated from GigE bandwidth |
| Bayer + ROI 2/3 crop | 2.1 MB | ~59 fps | ~0.1 ms (memcpy) | Calculated from GigE bandwidth |

### Benchmarks to Validate on Hardware

| Test | Target | Measurement | Pass Criteria |
|------|--------|-------------|---------------|
| Single cam FPS (RGB8, no ROI) | >=30 fps | `ros2 topic hz` | Sustained 30+ fps for 60s, 0 drops |
| Single cam FPS (Bayer, no ROI) | >=39 fps | `ros2 topic hz` | Sustained 39+ fps for 60s |
| Single cam FPS (Bayer + ROI) | >=55 fps | `ros2 topic hz` | Sustained 55+ fps for 60s |
| Frame size (Bayer, no ROI) | ~3.2 MB | `ros2 topic bw` | 3.0-3.3 MB per message |
| Frame size (Bayer + ROI) | ~2.1 MB | `ros2 topic bw` | 2.0-2.2 MB per message |
| 6-camera simultaneous | All publish | `ros2 topic hz` per camera | All 6 topics active, none at 0 fps |
| PTP timestamp sync | <1 us delta | Compare `header.stamp` across 2 cameras | Delta < 1 microsecond |
| Driver latency (Bayer) | <0.5 ms | Publish timestamp - frame arrival timestamp | < 0.5 ms p99 |
| Zero drops (10 min) | 0 drops | Driver log `[WARN] frame drop` count | Zero warns |
| PCAP roundtrip | Identical | Record pcap, replay, compare | Pixel-identical output |
| GPU memory (6 cams) | <500 MB | `nvidia-smi` | Driver VRAM < 500 MB |
| CPU usage (6 cams) | <2 cores | `htop` | Total driver CPU < 200% |

---

### Build

```bash
# CUDA node (opt-in, default ON in this branch)
colcon build --packages-select avt_vimba_camera \
  --cmake-args -DBUILD_CUDA_NODE=ON

# CPU-only (original mono_camera_node only)
colcon build --packages-select avt_vimba_camera \
  --cmake-args -DBUILD_CUDA_NODE=OFF
```

Dependencies: Vimba SDK (bundled `.so` for x86_64/arm64), CUDA toolkit, OpenCV 4.x, libpcap, ROS 2 Humble/Jazzy, `image_transport`, `camera_info_manager`.

### Launch

```bash
# Single camera -- safe/debug mode (RGB8)
ros2 launch avt_vimba_camera cuda_camera.launch.py \
  ip:=169.254.100.1 \
  publish_raw_bayer:=false \
  roi_enabled:=false \
  enable_ptp_sync:=false

# Single camera -- racing mode (Bayer + ROI)
ros2 launch avt_vimba_camera cuda_camera.launch.py \
  ip:=169.254.100.1 \
  publish_raw_bayer:=true \
  roi_enabled:=true \
  enable_ptp_sync:=true

# All 6 IAC cameras (link-local IPs 169.254.100.1-6)
ros2 launch avt_vimba_camera all_cameras_cuda.launch.py

# PCAP replay (no camera needed)
ros2 launch avt_vimba_camera cuda_camera.launch.py \
  enable_pcap:=true \
  pcap_file:=/path/to/capture.pcap
```

### 6-Camera IAC Layout

| Name | Default IP | TF Frame |
|------|------------|----------|
| front_left_center | 169.254.100.1 | front_left_center_camera |
| front_right_center | 169.254.100.2 | front_right_center_camera |
| front_left_far | 169.254.100.3 | front_left_far_camera |
| front_right_far | 169.254.100.4 | front_right_far_camera |
| rear_left | 169.254.100.5 | rear_left_camera |
| rear_right | 169.254.100.6 | rear_right_camera |

### Related

Full perception stack documentation including end-to-end test procedures: [`race_perception` README](https://github.com/ckwolfe/perception/blob/v0.3.0/race_perception/README.md#camera-driver-fixes-what-we-tried-why-and-whats-left)
