const std = @import("std");

// Although this function looks imperative, it does not perform the build
// directly and instead it mutates the build graph (`b`) that will be then
// executed by an external runner. The functions in `std.Build` implement a DSL
// for defining build steps and express dependencies between them, allowing the
// build runner to parallelize the build automatically (and the cache system to
// know when a step doesn't need to be re-run).
pub fn build(b: *std.Build) void {
    // Standard target options allow the person running `zig build` to choose
    // what target to build for. Here we do not override the defaults, which
    // means any target is allowed, and the default is native. Other options
    // for restricting supported target set are available.
    const target = b.standardTargetOptions(.{});
    // Standard optimization options allow the person running `zig build` to select
    // between Debug, ReleaseSafe, ReleaseFast, and ReleaseSmall. Here we do not
    // set a preferred release mode, allowing the user to decide how to optimize.
    const optimize = b.standardOptimizeOption(.{});
    // It's also possible to define more custom flags to toggle optional features
    // of this build script using `b.option()`. All defined flags (including
    // target and optimize options) will be listed when running `zig build --help`
    // in this directory.

    // This creates a module, which represents a collection of source files alongside
    // some compilation options, such as optimization mode and linked system libraries.
    // Zig modules are the preferred way of making Zig code available to consumers.
    // addModule defines a module that we intend to make available for importing
    // to our consumers. We must give it a name because a Zig package can expose
    // multiple modules and consumers will need to be able to specify which
    // module they want to access.
    // const mod = b.addModule("src_3", .{
    //     // The root source file is the "entry point" of this module. Users of
    //     // this module will only be able to access public declarations contained
    //     // in this file, which means that if you have declarations that you
    //     // intend to expose to consumers that were defined in other files part
    //     // of this module, you will have to make sure to re-export them from
    //     // the root file.
    //     .root_source_file = b.path("src/root.zig"),
    //     // Later on we'll use this module as the root module of a test executable
    //     // which requires us to specify a target.
    //     .target = target,
    // });

    // Here we define an executable. An executable needs to have a root module
    // which needs to expose a `main` function. While we could add a main function
    // to the module defined above, it's sometimes preferable to split business
    // logic and the CLI into two separate modules.
    //
    // If your goal is to create a Zig library for others to use, consider if
    // it might benefit from also exposing a CLI tool. A parser library for a
    // data serialization format could also bundle a CLI syntax checker, for example.
    //
    // If instead your goal is to create an executable, consider if users might
    // be interested in also being able to embed the core functionality of your
    // program in their own executable in order to avoid the overhead involved in
    // subprocessing your CLI tool.
    //
    // If neither case applies to you, feel free to delete the declaration you
    // don't need and to put everything under a single module.
    const exe = b.addExecutable(.{
        .name = "src_3",
        .root_module = b.createModule(.{
            // b.createModule defines a new module just like b.addModule but,
            // unlike b.addModule, it does not expose the module to consumers of
            // this package, which is why in this case we don't have to give it a name.
            .root_source_file = b.path("src/main.zig"),
            // Target and optimization levels must be explicitly wired in when
            // defining an executable or library (in the root module), and you
            // can also hardcode a specific target for an executable or library
            // definition if desireable (e.g. firmware for embedded devices).
            .target = target,
            .optimize = optimize,
            // List of modules available for import in source files part of the
            // root module.
            .imports = &.{
                // Here "src_3" is the name you will use in your source code to
                // import this module (e.g. `@import("src_3")`). The name is
                // repeated because you are allowed to rename your imports, which
                // can be extremely useful in case of collisions (which can happen
                // importing modules from different packages).
                // .{ .name = "src_3", .module = mod },
            },
        }),
    });

    exe.root_module.link_libc = true;
    exe.root_module.addIncludePath(b.path("include"));
    exe.root_module.addCSourceFile(.{
        .file = b.path("auvs/trap_sim/auv.c"),
    });

    // This declares intent for the executable to be installed into the
    // install prefix when running `zig build` (i.e. when executing the default
    // step). By default the install prefix is `zig-out/` but can be overridden
    // by passing `--prefix` or `-p`.
    b.installArtifact(exe);

    // This creates a top level step. Top level steps have a name and can be
    // invoked by name when running `zig build` (e.g. `zig build run`).
    // This will evaluate the `run` step rather than the default step.
    // For a top level step to actually do something, it must depend on other
    // steps (e.g. a Run step, as we will see in a moment).
    const run_step = b.step("run", "Run the app");

    // This creates a RunArtifact step in the build graph. A RunArtifact step
    // invokes an executable compiled by Zig. Steps will only be executed by the
    // runner if invoked directly by the user (in the case of top level steps)
    // or if another step depends on it, so it's up to you to define when and
    // how this Run step will be executed. In our case we want to run it when
    // the user runs `zig build run`, so we create a dependency link.
    const run_cmd = b.addRunArtifact(exe);
    run_step.dependOn(&run_cmd.step);

    // By making the run step depend on the default step, it will be run from the
    // installation directory rather than directly from within the cache directory.
    run_cmd.step.dependOn(b.getInstallStep());

    // This allows the user to pass arguments to the application in the build
    // command itself, like this: `zig build run -- arg1 arg2 etc`
    if (b.args) |args| {
        run_cmd.addArgs(args);
    }

    // Creates an executable that will run `test` blocks from the provided module.
    // Here `mod` needs to define a target, which is why earlier we made sure to
    // set the releative field.
    // const mod_tests = b.addTest(.{
    //     .root_module = mod,
    // });

    // A run step that will run the test executable.
    // const run_mod_tests = b.addRunArtifact(mod_tests);

    // Creates an executable that will run `test` blocks from the executable's
    // root module. Note that test executables only test one module at a time,
    // hence why we have to create two separate ones.
    const exe_tests = b.addTest(.{
        .root_module = exe.root_module,
    });

    // A run step that will run the second test executable.
    const run_exe_tests = b.addRunArtifact(exe_tests);

    // A top level step for running all tests. dependOn can be called multiple
    // times and since the two run steps do not depend on one another, this will
    // make the two of them run in parallel.
    const test_step = b.step("test", "Run tests");
    // test_step.dependOn(&run_mod_tests.step);
    test_step.dependOn(&run_exe_tests.step);

    // Just like flags, top level steps are also listed in the `--help` menu.
    //
    // The Zig build system is entirely implemented in userland, which means
    // that it cannot hook into private compiler APIs. All compilation work
    // orchestrated by the build system will result in other Zig compiler
    // subcommands being invoked with the right flags defined. You can observe
    // these invocations when one fails (or you pass a flag to increase
    // verbosity) to validate assumptions and diagnose problems.
    //
    // Lastly, the Zig build system is relatively simple and self-contained,
    // and reading its source code will allow you to master it.


    const lib_auv_sleep_sim = b.addLibrary(.{
        .name = "auv_sleep_sim",
        .root_module = b.createModule(.{
            .target = b.graph.host,
            .optimize = optimize,
        }),
        .linkage = .dynamic,
    });
    lib_auv_sleep_sim.root_module.link_libc = true;
    lib_auv_sleep_sim.root_module.addCSourceFile(.{
        .file = b.path("auvs/sleep_sim/auv.c"),
        .flags = &.{"-fPIC"},
    });
    b.installArtifact(lib_auv_sleep_sim);

    const lib_auv_proteus_sim = b.addLibrary(.{
        .name = "auv_proteus_sim",
        .root_module = b.createModule(.{
            .target = b.graph.host,
            .optimize = optimize,
        }),
        .linkage = .dynamic,
    });
    lib_auv_proteus_sim.root_module.link_libcpp = true;
    lib_auv_proteus_sim.root_module.addCSourceFile(.{
        .file = b.path("auvs/proteus_sim/auv.cpp"),
        .flags = &.{"-fPIC", "-std=c++17"},
    });
    b.installArtifact(lib_auv_proteus_sim);

    // the zed sdk exposes c++ types, so this one has to be built with the
    // system g++ to match its libstdc++ abi. off by default so that machines
    // without the sdk and cuda can still build everything else.
    const zed = b.option(bool, "zed", "Build the ZED camera plugin (requires ZED SDK and CUDA)") orelse false;
    const cxx = b.option([]const u8, "cxx", "System C++ compiler") orelse "g++";
    const zed_prefix = b.option([]const u8, "zed-prefix", "ZED SDK install prefix") orelse "/usr/local/zed";
    const cuda_prefix = b.option([]const u8, "cuda-prefix", "CUDA install prefix") orelse "/usr/local/cuda";
    if (zed) {
        const lib_auv_proteus_hwd = b.addSystemCommand(&.{cxx});
        lib_auv_proteus_hwd.addArgs(&.{"-shared", "-fPIC", "-std=c++17", "-Wall", "-Wextra", "-Werror"});
        lib_auv_proteus_hwd.addArg(if (optimize == .Debug) "-O0" else "-O2");
        if (optimize == .Debug)
            lib_auv_proteus_hwd.addArg("-g");

        lib_auv_proteus_hwd.addArg("-o");
        const lib_auv_proteus_hwd_so = lib_auv_proteus_hwd.addOutputFileArg("libauv_proteus_hwd.so");
        lib_auv_proteus_hwd.addFileArg(b.path("auvs/proteus_hwd/auv.cpp"));
        lib_auv_proteus_hwd.addFileInput(b.path("include/auv.h"));
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

        const install_lib_auv_proteus_hwd = b.addInstallFileWithDir(lib_auv_proteus_hwd_so, .lib, "libauv_proteus_hwd.so");
        b.getInstallStep().dependOn(&install_lib_auv_proteus_hwd.step);
    }

    if (b.option(bool, "hydrus", "Build the Stonefish hydrus_sim plugin") == true) {
        const stonefish_prefix = b.option([]const u8, "stonefish-prefix", "Stonefish install prefix") orelse
            @panic("-Dhydrus requires -Dstonefish-prefix");
        const lib_auv_hydrus_sim = b.addSystemCommand(&.{cxx});
        lib_auv_hydrus_sim.addArgs(&.{
            "-shared",
            "-fPIC",
            "-std=gnu++23",
            "-D_GNU_SOURCE",
            "-DBT_EULER_DEFAULT_ZYX",
            "-DBT_USE_DOUBLE_PRECISION",
            "-fopenmp",
        });
        lib_auv_hydrus_sim.addArg(b.fmt("-DSHADER_DIR_PATH=\"{s}/share/Stonefish/shaders/\"", .{stonefish_prefix}));
        lib_auv_hydrus_sim.addArg(if (optimize == .Debug) "-O0" else "-O2");
        if (optimize == .Debug)
            lib_auv_hydrus_sim.addArg("-g");

        lib_auv_hydrus_sim.addArg("-o");
        const lib_auv_hydrus_sim_so = lib_auv_hydrus_sim.addOutputFileArg("libauv_hydrus_sim.so");
        lib_auv_hydrus_sim.addFileArg(b.path("auvs/hydrus_sim/auv.cpp"));
        lib_auv_hydrus_sim.addFileInput(b.path("include/auv.h"));
        lib_auv_hydrus_sim.addFileInput(b.path("include/math.h"));
        lib_auv_hydrus_sim.addArgs(&.{
            "-I",
            b.pathFromRoot("include"),
            "-isystem",
            b.fmt("{s}/include/Stonefish", .{stonefish_prefix}),
            "-isystem",
            "/usr/include/SDL2",
            b.fmt("-L{s}/lib", .{stonefish_prefix}),
            b.fmt("-Wl,-rpath,{s}/lib", .{stonefish_prefix}),
            "-Wl,--no-undefined",
            "-lStonefish",
            "-lSDL2",
            "-lfreetype",
            "-lOpenGL",
            "-lGLX",
            "-lGLU",
            "-lgomp",
            "-pthread",
        });

        const install_lib_auv_hydrus_sim = b.addInstallFileWithDir(lib_auv_hydrus_sim_so, .lib, "libauv_hydrus_sim.so");
        b.getInstallStep().dependOn(&install_lib_auv_hydrus_sim.step);
        b.step("hydrus", "Build the Stonefish hydrus_sim plugin").dependOn(&install_lib_auv_hydrus_sim.step);
    }
}
