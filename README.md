## Build
```sh
zig build
```

## Build & Run
```sh
zig build run -- \
    ./zig-out/lib/libauv_sleep_sim.so \
    prequalify \
    ./auvs/sleep_sim/auv.json
```

## Hydrus sim
Requires a Stonefish install. From the repo root:

```sh
zig build -Dhydrus -Dstonefish-prefix=/usr/local
./zig-out/bin/src_3 ./zig-out/lib/libauv_hydrus_sim.so prequalify auvs/hydrus_sim/auv.json
```
