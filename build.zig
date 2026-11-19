const std = @import("std");

pub fn build(b: *std.Build) !void {
    const target = b.standardTargetOptions(.{});

    const export_scene_catalog_emit = b.createModule(.{
        .root_source_file = b.path("src/simulator/catalog_emit.zig"),
        .target = b.graph.host,
    });
    const export_scene = b.addExecutable(.{
        .name = "export_scene",
        .root_module = b.createModule(.{
            .root_source_file = b.path("tools/export_scene/main.zig"),
            .target = b.graph.host,
            .optimize = .ReleaseSafe,
            .imports = &.{
                .{ .name = "catalog_emit", .module = export_scene_catalog_emit },
            },
        }),
    });
    const run_export_scene = b.addRunArtifact(export_scene);
    run_export_scene.addArg("--preset");
    run_export_scene.addArg("robosub");
    run_export_scene.setCwd(b.path("."));
    b.step("export-scene", "Regenerate scenes.bin").dependOn(&run_export_scene.step);

    const optimize = b.standardOptimizeOption(.{});
    const cxx = b.option([]const u8, "cxx", "System C++ compiler") orelse "g++";
    const zed_prefix = b.option([]const u8, "zed-prefix", "ZED SDK install prefix") orelse "/usr/local/zed";
    const cuda_prefix = b.option([]const u8, "cuda-prefix", "CUDA install prefix") orelse "/usr/local/cuda";
    const stdcxx_path = b.option(
        []const u8,
        "stdcxx-path",
        "System libstdc++.so used by ZED C++ objects",
    ) orelse "/usr/lib/x86_64-linux-gnu/libstdc++.so.6";

    const exe = b.addExecutable(.{
        .name = "src_3",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{},
        }),
    });

    const simulator = b.option(bool, "simulator", "Compile the native Zig/MuJoCo simulator") orelse false;
    const build_options = b.addOptions();
    build_options.addOption(bool, "simulator", simulator);
    exe.root_module.addOptions("build_options", build_options);
    var mujoco_build_step: ?*std.Build.Step = null;
    if (simulator) {
        const cmake_configure = b.addSystemCommand(&.{
            "cmake",
            "-S",
            "vendor/mujoco",
            "-B",
            "vendor/mujoco/build",
            "-DMUJOCO_BUILD_EXAMPLES=OFF",
            "-DMUJOCO_BUILD_SIMULATE=OFF",
            "-DMUJOCO_BUILD_STUDIO=OFF",
            "-DMUJOCO_BUILD_TESTS=OFF",
            "-DMUJOCO_TEST_PYTHON_UTIL=OFF",
        });
        const cmake_build = b.addSystemCommand(&.{
            "cmake",
            "--build",
            "vendor/mujoco/build",
            "--target",
            "mujoco",
            "--parallel",
        });
        cmake_build.step.dependOn(&cmake_configure.step);
        mujoco_build_step = &cmake_build.step;

        const headers = b.path("vendor/mujoco/include");
        const library = b.path("vendor/mujoco/build/lib/libmujoco.so");
        const mujoco_c = b.addTranslateC(.{
            .root_source_file = b.path("vendor/mujoco/include/mujoco/mujoco.h"),
            .target = target,
            .optimize = optimize,
        });
        mujoco_c.addIncludePath(headers);
        exe.root_module.addImport("mujoco_c", mujoco_c.createModule());
        exe.root_module.addObjectFile(library);
        exe.root_module.addRPath(library.dirname());
        exe.root_module.addRPathSpecial("$ORIGIN/../../vendor/mujoco/build/lib");
        exe.step.dependOn(mujoco_build_step.?);
    }

    exe.root_module.link_libc = true;
    exe.root_module.addIncludePath(b.path("include"));
    b.installArtifact(exe);
    const raylib_dep = b.dependency("raylib_zig", .{
        .target = target,
        .optimize = optimize,
    });
    const raylib = raylib_dep.module("raylib");
    if (simulator) exe.root_module.addImport("raylib", raylib);
    const visualizer = b.createModule(.{
        .root_source_file = b.path("src/Visualizer.zig"),
        .target = target,
        .optimize = optimize,
        .link_libc = true,
        .imports = &.{.{ .name = "raylib", .module = raylib }},
    });
    visualizer.addIncludePath(b.path("include"));
    const viewer_mod = b.createModule(.{
        .root_source_file = b.path("tools/viewer/main.zig"),
        .target = target,
        .optimize = optimize,
        .link_libc = true,
        .imports = &.{
            .{ .name = "raylib", .module = raylib },
            .{ .name = "Visualizer", .module = visualizer },
        },
    });
    const viewer = b.addExecutable(.{
        .name = "viewer",
        .root_module = viewer_mod,
    });

    const viewer_zed = b.option(bool, "viewer-zed", "Enable ZED stream/SVO video in the Zig viewer") orelse false;
    const viewer_gstreamer = b.option(bool, "viewer-gstreamer", "Enable standard RTP video in the viewer using GStreamer") orelse false;
    if (viewer_zed and viewer_gstreamer) @panic("-Dviewer-zed and -Dviewer-gstreamer select different video backends");
    if (viewer_zed) {
        const compile_video = b.addSystemCommand(&.{
            cxx,
            "-c",
            "-fPIC",
            "-std=c++17",
            "-Wall",
            "-Wextra",
            "-Werror",
        });
        compile_video.addArgs(&.{
            "-isystem",
            b.fmt("{s}/include", .{zed_prefix}),
            "-isystem",
            b.fmt("{s}/include", .{cuda_prefix}),
            "-o",
        });
        const video_object = compile_video.addOutputFileArg("viewer_video.o");
        compile_video.addFileArg(b.path("tools/viewer/video.cpp"));
        viewer.root_module.addObjectFile(video_object);
        viewer.root_module.addLibraryPath(.{ .cwd_relative = b.fmt("{s}/lib", .{zed_prefix}) });
        viewer.root_module.addLibraryPath(.{ .cwd_relative = b.fmt("{s}/lib64", .{cuda_prefix}) });
        viewer.root_module.linkSystemLibrary("sl_zed", .{});
        viewer.root_module.linkSystemLibrary("cuda", .{});
        viewer.root_module.linkSystemLibrary("cudart", .{});
        viewer.root_module.linkSystemLibrary("gcc_s", .{ .use_pkg_config = .no });
        viewer.root_module.addObjectFile(.{ .cwd_relative = stdcxx_path });
    } else if (viewer_gstreamer) {
        viewer.root_module.link_libcpp = true;
        viewer.root_module.addCSourceFile(.{
            .file = b.path("tools/viewer/video_gstreamer.cpp"),
            .flags = &.{ "-std=c++17", "-Wall", "-Wextra", "-Werror" },
        });
        viewer.root_module.linkSystemLibrary("gstreamer-app-1.0", .{ .use_pkg_config = .force });
        viewer.root_module.linkSystemLibrary("gstreamer-video-1.0", .{ .use_pkg_config = .force });
    } else {
        viewer.root_module.addCSourceFile(.{
            .file = b.path("tools/viewer/video_stub.cpp"),
            .flags = &.{ "-std=c++17", "-Wall", "-Wextra", "-Werror" },
        });
        viewer.root_module.linkSystemLibrary("stdc++", .{});
    }

    const install_viewer = b.addInstallArtifact(viewer, .{});
    const viewer_step = b.step("viewer", "Build the Zig telemetry viewer");
    viewer_step.dependOn(&install_viewer.step);
    const run_step = b.step("run", "Run the app");
    const run_cmd = b.addRunArtifact(exe);
    run_step.dependOn(&run_cmd.step);

    run_cmd.step.dependOn(b.getInstallStep());

    run_cmd.addPassthruArgs();

    const test_module = b.createModule(.{
        .root_source_file = b.path("src/main.zig"),
        .target = target,
        .optimize = optimize,
        .link_libc = true,
    });
    test_module.addIncludePath(b.path("include"));
    for (exe.root_module.import_table.keys(), exe.root_module.import_table.values()) |name, module| test_module.addImport(name, module);
    for (exe.root_module.link_objects.items) |object| if (object == .static_path) try test_module.link_objects.append(b.allocator, object);
    for (exe.root_module.rpaths.items) |path| try test_module.rpaths.append(b.allocator, path);
    const exe_tests = b.addTest(.{ .root_module = test_module });
    if (mujoco_build_step) |step| exe_tests.step.dependOn(step);

    const run_exe_tests = b.addRunArtifact(exe_tests);

    // Independent test suites run in parallel.
    const test_step = b.step("test", "Run tests");
    test_step.dependOn(&run_exe_tests.step);

    const integration = b.addSystemCommand(&.{"python3"});
    integration.addFileArg(b.path("tests/serial_integration.py"));
    integration.addArtifactArg(exe);
    test_step.dependOn(&integration.step);

    for ([_][]const u8{ "pending", "slow", "init_fail", "frame_fail" }, 0..) |name, mode| {
        const fixture = b.addLibrary(.{ .name = b.fmt("control_sensor_{s}", .{name}), .linkage = .dynamic, .root_module = b.createModule(.{ .target = target, .optimize = optimize, .link_libc = true }) });
        fixture.root_module.addCSourceFile(.{ .file = b.path("tests/control_sensor.c"), .flags = &.{b.fmt("-DTEST_MODE={d}", .{mode})} });
        integration.addArtifactArg(fixture);
    }

    const firmware_tests = b.addExecutable(.{
        .name = "firmware_test",
        .root_module = b.createModule(.{ .target = target, .optimize = optimize, .link_libc = true, .link_libcpp = true }),
    });
    firmware_tests.root_module.addIncludePath(b.path("tests/firmware_stubs"));
    firmware_tests.root_module.addCSourceFile(.{ .file = b.path("tests/firmware_test.cpp"), .flags = &.{ "-std=c++17", "-Wall", "-Wextra", "-Werror" } });
    const run_firmware_tests = b.addRunArtifact(firmware_tests);
    test_step.dependOn(&run_firmware_tests.step);

    const viewer_ui_tests = b.addTest(.{ .root_module = visualizer });
    const run_viewer_ui_tests = b.addRunArtifact(viewer_ui_tests);
    test_step.dependOn(&run_viewer_ui_tests.step);

    const map_tests = b.addExecutable(.{
        .name = "object_map_test",
        .root_module = b.createModule(.{
            .target = target,
            .optimize = optimize,
            .link_libc = true,
            .link_libcpp = true,
        }),
    });
    map_tests.root_module.addCSourceFile(.{ .file = b.path("tests/object_map_test.cpp"), .flags = &.{ "-std=c++17", "-Wall", "-Wextra", "-Werror" } });
    map_tests.root_module.addCSourceFile(.{ .file = b.path("auvs/proteus_hwd/object_map.cpp"), .flags = &.{ "-std=c++17", "-Wall", "-Wextra", "-Werror" } });
    const run_map_tests = b.addRunArtifact(map_tests);
    test_step.dependOn(&run_map_tests.step);

    if (viewer_gstreamer) {
        const rtp_test_mod = b.createModule(.{
            .target = target,
            .optimize = optimize,
            .link_libc = true,
            .link_libcpp = true,
        });
        for ([_][]const u8{ "tests/rtp_stream_test.cpp", "tools/viewer/video_gstreamer.cpp", "auvs/proteus_hwd/rtp_relay.cpp" }) |path| {
            rtp_test_mod.addCSourceFile(.{ .file = b.path(path), .flags = &.{ "-std=c++17", "-Wall", "-Wextra", "-Werror" } });
        }
        rtp_test_mod.linkSystemLibrary("gstreamer-app-1.0", .{ .use_pkg_config = .force });
        rtp_test_mod.linkSystemLibrary("gstreamer-video-1.0", .{ .use_pkg_config = .force });
        const rtp_tests = b.addExecutable(.{ .name = "rtp_stream_test", .root_module = rtp_test_mod });
        const run_rtp_tests = b.addRunArtifact(rtp_tests);
        test_step.dependOn(&run_rtp_tests.step);
    }

    const arduino_cli = b.option([]const u8, "arduino-cli", "Arduino CLI executable") orelse "arduino-cli";
    const arduino_fqbn = b.option([]const u8, "arduino-fqbn", "Arduino board FQBN") orelse "arduino:avr:uno";
    const arduino_compile = b.addSystemCommand(&.{ arduino_cli, "compile", "--fqbn", arduino_fqbn });
    arduino_compile.addDirectoryArg(b.path("arduino/sketches/Proteus"));
    // Incremental watch for Arduino sketch edits (API varies by Zig version).
    if (@hasDecl(std.Build.Step, "addDirectoryWatchInput")) {
        _ = try arduino_compile.step.addDirectoryWatchInput(b.path("arduino/sketches/Proteus"));
    }

    const arduino_step = b.step("arduino", "Compile the Proteus Arduino sketch");
    arduino_step.dependOn(&arduino_compile.step);

    const lib_auv_stationary_sim = b.addLibrary(.{
        .name = "auv_stationary_sim",
        .root_module = b.createModule(.{
            .target = b.graph.host,
            .optimize = optimize,
        }),
        .linkage = .dynamic,
    });
    lib_auv_stationary_sim.root_module.link_libc = true;
    lib_auv_stationary_sim.root_module.addCSourceFile(.{
        .file = b.path("auvs/stationary_sim/auv.c"),
        .flags = &.{"-fPIC"},
    });
    b.installArtifact(lib_auv_stationary_sim);
    const test_paths = b.addOptions();
    test_paths.addOptionPath("sensor", lib_auv_stationary_sim.getEmittedBin());
    test_module.addOptions("test_paths", test_paths);
    integration.addArtifactArg(lib_auv_stationary_sim);
    integration.addArg(if (simulator) "simulated" else "hardware");
    const fast_sensor = b.addLibrary(.{
        .name = "auv_fast_sensor",
        .linkage = .dynamic,
        .root_module = b.createModule(.{ .target = target, .optimize = optimize, .link_libc = true }),
    });
    fast_sensor.root_module.addCSourceFile(.{ .file = b.path("auvs/stationary_sim/auv.c"), .flags = &.{ "-fPIC", "-DFRAME_INTERVAL_NS=0" } });
    integration.addArtifactArg(fast_sensor);
    test_paths.addOptionPath("fast_sensor", fast_sensor.getEmittedBin());

    const complete_sensor = b.addLibrary(.{
        .name = "control_sensor_complete",
        .linkage = .dynamic,
        .root_module = b.createModule(.{ .target = target, .optimize = optimize, .link_libc = true }),
    });
    complete_sensor.root_module.addCSourceFile(.{ .file = b.path("tests/control_sensor.c"), .flags = &.{"-DTEST_MODE=4"} });
    integration.addArtifactArg(complete_sensor);

    const deadline_sensor = b.addLibrary(.{
        .name = "auv_deadline_sensor",
        .linkage = .dynamic,
        .root_module = b.createModule(.{ .target = target, .optimize = optimize, .link_libc = true }),
    });
    deadline_sensor.root_module.addCSourceFile(.{ .file = b.path("auvs/stationary_sim/auv.c"), .flags = &.{"-DFRAME_INTERVAL_NS=2000000000ULL"} });
    integration.addArtifactArg(deadline_sensor);

    const zed = b.option(bool, "zed", "Build the ZED camera plugin (requires ZED SDK and CUDA)") orelse false;
    const zed_rtp = b.option(bool, "zed-rtp", "Enable standard RTP relay in the ZED plugin (SDK >= 5.4 and GStreamer)") orelse false;
    if (zed_rtp and !zed) @panic("-Dzed-rtp requires -Dzed");
    if (zed) {
        const lib_auv_proteus_hwd = b.addSystemCommand(&.{cxx});
        lib_auv_proteus_hwd.addArgs(&.{ "-shared", "-fPIC", "-std=c++17", "-Wall", "-Wextra", "-Werror" });
        lib_auv_proteus_hwd.addArg(if (optimize == .debug) "-O0" else "-O2");
        if (optimize == .debug)
            lib_auv_proteus_hwd.addArg("-g");

        lib_auv_proteus_hwd.addArg("-o");
        const lib_auv_proteus_hwd_so = lib_auv_proteus_hwd.addOutputFileArg("libauv_proteus_hwd.so");
        if (zed_rtp) {
            lib_auv_proteus_hwd.addArg("-DAUV_STANDARD_RTP");
            lib_auv_proteus_hwd.addFileArg(b.path("auvs/proteus_hwd/rtp_relay.cpp"));
            lib_auv_proteus_hwd.addFileInput(b.path("auvs/proteus_hwd/rtp_relay.hpp"));
        }
        lib_auv_proteus_hwd.addFileArg(b.path("auvs/proteus_hwd/auv.cpp"));
        lib_auv_proteus_hwd.addFileArg(b.path("auvs/proteus_hwd/object_map.cpp"));
        lib_auv_proteus_hwd.addFileInput(b.path("auvs/proteus_hwd/object_map.hpp"));
        lib_auv_proteus_hwd.addFileInput(b.path("auvs/proteus_hwd/coordinates.hpp"));
        lib_auv_proteus_hwd.addFileInput(b.path("include/auv.h"));
        lib_auv_proteus_hwd.addFileInput(b.path("include/object_classes.h"));
        lib_auv_proteus_hwd.addFileInput(b.path("include/math.h"));
        lib_auv_proteus_hwd.addArgs(&.{
            "-isystem",
            b.fmt("{s}/include", .{zed_prefix}),
            "-isystem",
            b.fmt("{s}/include", .{cuda_prefix}),
            b.fmt("-L{s}/lib", .{zed_prefix}),
            b.fmt("-L{s}/lib64", .{cuda_prefix}),
            b.fmt("-Wl,-rpath,{s}/lib", .{zed_prefix}),
            b.fmt("-Wl,-rpath,{s}/lib64", .{cuda_prefix}),
            "-Wl,--no-undefined",
            "-lsl_zed",
            "-lcudart",
            "-lcuda",
            "-pthread",
        });

        if (zed_rtp) {
            const flags = b.run(&.{ "pkg-config", "--cflags", "--libs", "gstreamer-app-1.0" });
            var parts = std.mem.tokenizeAny(u8, flags, " \t\r\n");
            while (parts.next()) |part| lib_auv_proteus_hwd.addArg(part);
        }

        const install_lib_auv_proteus_hwd = b.addInstallFileWithDir(lib_auv_proteus_hwd_so, .lib, "libauv_proteus_hwd.so");
        b.getInstallStep().dependOn(&install_lib_auv_proteus_hwd.step);
    }

    const cmake_configure = b.addSystemCommand(&.{
        "cmake",
        "-B",
        "build",
        "-DCMAKE_EXPORT_COMPILE_COMMANDS=ON",
    });
    if (@hasDecl(std.Build.Step, "addWatchInput")) {
        try cmake_configure.step.addWatchInput(b.path("CMakeLists.txt"));
    }

    const cpu_count = std.Thread.getCpuCount() catch 1;
    const cmake_build = b.addSystemCommand(&.{
        "cmake",
        "--build",
        "build",
        "--parallel",
        b.fmt("{d}", .{cpu_count}),
        "--target",
    });
    cmake_build.step.dependOn(&cmake_configure.step);

    const stonefish = b.option(bool, "stonefish", "Build the Stonefish simulator plugin") orelse false;
    if (stonefish) {
        const cmake_target = "auv_stonefish_sim";
        cmake_build.addArg(cmake_target);

        if (@hasDecl(std.Build.Step, "addDirectoryWatchInput")) {
            _ = try cmake_build.step.addDirectoryWatchInput(b.path("auvs/stonefish_sim"));
        }

        const cmake_dynlib_file = b.fmt("lib{s}{s}", .{ cmake_target, target.result.dynamicLibSuffix() });
        const cmake_dynlib_path = b.path(b.fmt("build/{s}", .{cmake_dynlib_file}));

        const install_stonefish_sim = b.addInstallFileWithDir(cmake_dynlib_path, .lib, cmake_dynlib_file);
        install_stonefish_sim.step.dependOn(&cmake_build.step);

        b.getInstallStep().dependOn(&install_stonefish_sim.step);
    }

    cmake_build.step.dependOn(&cmake_configure.step);
}
