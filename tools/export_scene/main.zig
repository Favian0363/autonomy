const std = @import("std");
const catalog_emit = @import("catalog_emit");

pub fn main(init: std.process.Init) !void {
    const allocator = init.arena.allocator();
    const io = init.io;
    const arg_slice = try init.minimal.args.toSlice(allocator);

    const usage =
        \\Usage:
        \\  export_scene <file.blend> [--collection robosub_course]
        \\
        \\Custom properties on exported mesh objects:
        \\  sim_id, sim_cls, sim_tracked, sim_collisions
        \\Optional box fallback: sim_hx, sim_hy, sim_hz
        \\
    ;

    var blend_path: ?[]const u8 = null;
    var collection: []const u8 = "robosub_course";

    var index: usize = 1;
    while (index < arg_slice.len) : (index += 1) {
        const arg = arg_slice[index];
        if (std.mem.eql(u8, arg, "--help") or std.mem.eql(u8, arg, "-h")) {
            std.debug.print("{s}", .{usage});
            return;
        } else if (std.mem.eql(u8, arg, "--collection")) {
            index += 1;
            if (index >= arg_slice.len) return error.MissingCollectionName;
            collection = arg_slice[index];
        } else if (blend_path == null) {
            blend_path = arg;
        } else {
            std.debug.print("{s}", .{usage});
            return error.InvalidArguments;
        }
    }

    const path = blend_path orelse {
        std.debug.print("{s}", .{usage});
        return error.MissingInput;
    };

    var file = try catalog_emit.BlendFile.load(allocator, io, path);
    defer file.deinit();
    const objects = try catalog_emit.sourcesFromBlend(allocator, &file, collection);
    defer catalog_emit.freeSources(allocator, objects, true);

    const catalog = try catalog_emit.writeCatalog(allocator, collection, objects);
    defer allocator.free(catalog);

    const dir = std.Io.Dir.cwd();
    try dir.writeFile(io, .{ .sub_path = "src/simulator/scenes.bin", .data = catalog });
    std.log.info("wrote scenes.bin", .{});
}