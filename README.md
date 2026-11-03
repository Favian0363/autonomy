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

Headless and faster than real time (for batch runs and machines without a display), stopping
after 60 simulated seconds and writing the pose of every frame to `trace.csv`:

```sh
AUV_SIM_HEADLESS=1 AUV_SIM_REALTIME=0 AUV_SIM_DURATION=60 AUV_SIM_TRACE=trace.csv \
zig build -Dstonefish run -- \
    ./zig-out/lib/libauv_stonefish_sim.so  \
    prequalify_robosub \
    ./auvs/stonefish_sim/auv.json
```

Headless runs skip the camera sensor (Stonefish needs graphics for it), so frames have no 2D boxes.

Waypoint test: the `waypoints` mission makes a random course from a seed (sharp turns, points
behind the robot, depth changes; the same seed always gives the same course and the same run)
and drives to each point in order. `AUV_WAYPOINTS_SEED` picks the course (default 1) and
`AUV_WAYPOINTS_COUNT` its length (default 8). Each point must be reached (within 1 m) before the
timeout and then held for 5 s. The mission prints one `waypoint,...` line per point (time to
reach, overshoot, final distance, largest roll and pitch) and exits with an error if any point is
missed or the simulation diverges:

```sh
AUV_WAYPOINTS_SEED=1 \
zig build -Dstonefish run -- \
    ./zig-out/lib/libauv_stonefish_sim.so  \
    waypoints \
    ./auvs/stonefish_sim/auv.json
```

Same test headless, and with frames like the real robot's: the fault layer
(`libauv_fault_layer.so`) loads the simulator and passes only 30 frames per second to the
mission, each one 65 ms old, which is what the Jetson measured. The delay is rounded up to whole
frames, so 65 ms becomes about 67 ms; the layer prints the delay it applied. With
`AUV_FAULT_FRESH_IMU=1` the IMU fields stay current while the rest is late, as when the IMU is
read at the current time:

```sh
AUV_SIM_HEADLESS=1 AUV_SIM_REALTIME=0 AUV_WAYPOINTS_SEED=1 \
AUV_FAULT_TARGET=./zig-out/lib/libauv_stonefish_sim.so AUV_FAULT_RATE_HZ=30 AUV_FAULT_DELAY_MS=65 \
zig build -Dstonefish run -- \
    ./zig-out/lib/libauv_fault_layer.so  \
    waypoints \
    ./auvs/stonefish_sim/auv.json
```

Many courses at once: runs seeds 1..N under each condition (`perfect`, `jetson65`, `jetson115`,
or `delay<ms>` for 30 frames per second and any delay), several at a time, headless, and prints
a summary per condition. It writes one log per run and `results.csv` (one row per waypoint) to
`--out` (keep it out of git), prints the command to watch the first failure, and exits with an
error if any run failed:

```sh
zig build -Dstonefish
python3 tests/sim/run_courses.py --seeds 20 --out sim_results
python3 tests/sim/run_courses.py --seeds 30 --conditions delay0 delay99 delay198 delay297 --out sweep
```

## Build & Run: Proteus hardware (ZED 2i + Arduino)

```sh
zig build -Dzed
zig build -Dzed run -- \
    ./zig-out/lib/libauv_proteus_hwd.so \
    prequalify_coco \
    ./auvs/proteus_hwd/auv.json
```



