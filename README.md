# Autonomy

Requires Zig 0.17+, C/C++ toolchain, CMake, libc, pkg-config, and Python 3.

```sh
sudo apt install build-essential pkg-config python3 \
  cmake \
  libsdl2-dev libgl1-mesa-dev libx11-dev libxcursor-dev \
  libxinerama-dev libxrandr-dev libxi-dev
```

## Build, test, run

```sh
zig build
zig build test
zig build run
```

## Native simulator

```sh
git submodule update --init vendor/mujoco
zig build -Dsimulator
zig build test -Dsimulator
zig build run -Dsimulator
zig build run -Dsimulator -- --headless
```

Scene geometry comes from the embedded catalog `src/simulator/scenes.bin`. `sim_scene` selects a preset **name** inside that catalog (`robot_only`, `robosub_course`, etc.). Robots always come from embedded `robots.bin`. Refresh the committed catalog from Blender:

```sh
zig build export-scene
./zig-out/bin/export_scene path/to/course.blend --collection robosub_course
```

## ZED sensor plugin

```sh
zig build -Dzed \
  -Dzed-prefix=/usr/local/zed \
  -Dcuda-prefix=/usr/local/cuda
zig build run -Dzed
```

## Standalone viewer for hardware or headless simulation

```sh
zig build viewer
./zig-out/bin/viewer --video none
```

```sh
zig build viewer -Dviewer-zed
./zig-out/bin/viewer --video 127.0.0.1:5610
```

```sh
sudo apt install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad gstreamer1.0-libav
zig build viewer -Dviewer-gstreamer
./zig-out/bin/viewer --video rtp://0.0.0.0:5612
```

## Legacy Stonefish plugin

```sh
zig build -Dstonefish
```
