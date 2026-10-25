## Build & Run: libauv_stationary_sim

```sh
zig build run -- \
    ./zig-out/lib/libauv_stationary_sim.so \
    prequalify_coco \
    ./auvs/stationary_sim/auv.json
```

## Build & Run: teleop mission

Build the sender and simulator:

```sh
zig build teleop-sender
zig build
```

Run the teleop mission:

```sh
zig build run -- \
    ./zig-out/lib/libauv_stationary_sim.so \
    teleop \
    ./auvs/stationary_sim/auv.json
```

In another terminal, start the sender:

```sh
./zig-out/bin/teleop_sender 8080
```

## Build & Run: viewer

The viewer needs the ZED SDK and CUDA. CMake fetches raylib.

```sh
cmake -S tools/viewer -B tools/viewer/build
cmake --build tools/viewer/build
./tools/viewer/build/viewer
```

It listens for binary telemetry on UDP 5600 and the ZED H264 stream at `127.0.0.1:5610`. With the window focused, it sends the teleop goal to `127.0.0.1:8080`.

```sh
./tools/viewer/build/viewer [telemetry_port] [video] [teleop_port]
```

Hold keys to move the goal by 0.05 every 50 ms: W/S move X, A/D move Y, R/F move Z, Q/E change roll, and the arrow keys change pitch and yaw.

## Build & Run: libauv_stonefish_sim

```sh
zig build -Dstonefish run -- \
    ./zig-out/lib/libauv_stonefish_sim.so  \
    prequalify_robosub \
    ./auvs/stonefish_sim/auv.json
```

## Build & Run: Proteus hardware (ZED 2i + Arduino)

```sh
zig build -Dzed
zig build -Dzed run -- \
    ./zig-out/lib/libauv_proteus_hwd.so \
    prequalify_coco \
    ./auvs/proteus_hwd/auv.json
```



