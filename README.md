# Autonomy

Needs Zig 0.17+, CMake, and a C/C++ toolchain. On Debian/Ubuntu:

```sh
sudo apt install build-essential cmake pkg-config python3 \
  libsdl2-dev libgl1-mesa-dev libx11-dev libxcursor-dev \
  libxinerama-dev libxrandr-dev libxi-dev
```

## Hardware (Jetson + Proteus)

**On the vehicle** (ZED SDK under `/usr/local/zed`, CUDA under `/usr/local/cuda`):

```sh
zig build -Dzed -Dzed-prefix=/usr/local/zed -Dcuda-prefix=/usr/local/cuda
./zig-out/bin/src_3
```

Use the **Proteus** profile, serial port, and `libauv_proteus_hwd.so` paths from the viewer (UDP control **8080**, telemetry **5600**). Set **telemetry** on the Jetson to your ground-station IP `:5600` when the viewer is not on the same host.

**On a ground station** (telemetry )

```sh
zig build viewer
./zig-out/bin/viewer --video none --teleop-host JETSON_IP
```

With ZED RTP from the vehicle:

```sh
./zig-out/bin/viewer --video 127.0.0.1:5610 --teleop-host JETSON_IP
```

```sh
zig build test
```


## MuJoCo + embedded visualizer

```sh
git submodule update --init vendor/mujoco
zig build -Dsimulator
zig build run -Dsimulator
zig build run -Dsimulator -- --headless
zig build test -Dsimulator
```




## Stonefish (Legacy Simulator)

Run from the **repo root**.

```sh
git submodule update --init vendor/stonefish
zig build -Dstonefish
./zig-out/bin/src_3
```
