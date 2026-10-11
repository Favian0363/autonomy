
## Build & Run: libauv_stationary_sim
```sh
zig build run -- \
    ./zig-out/lib/libauv_stationary_sim.so \
    prequalify \
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

## Build & Run: libauv_stonefish_sim

```sh
zig build -Dstonefish run -- \
    ./zig-out/lib/libauv_stonefish_sim.so  \
    prequalify \
    ./auvs/stonefish_sim/auv.json
```
