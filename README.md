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

Environment layout is selected with `sim_scene_catalog` in mission config (see [`MissionConfig.zig`](src/MissionConfig.zig)):

| `sim_scene_catalog` | Behavior |
|---|---|
| `embedded_bin` (default) | Load committed [`src/simulator/scenes.bin`](src/simulator/scenes.bin) |
| `file` | Load `sim_scene_catalog_path` |

`sim_scene` still picks a **preset name** inside the catalog (`robosub_course`, `robot_only`, …). Scene and robot files are separate DNA documents (`database.loadScene`, `database.loadRobot`). Each file header is the reflection of the struct that was saved. Robots stay in `robots.bin`.

Blender is only for **mesh geometry** (`zig build export-mesh` → `.meshbin` for offline catalog authoring).

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
