const std = @import("std");
const blend = @import("blend_catalog");
const mesh_wire = @import("mesh_wire");

const help =
    \\Usage: export_mesh <file.blend> --object <name> -o <out.meshbin>
    \\
    \\Exports one Blender object's triangle mesh (no scene metadata). Commit the
    \\.meshbin under src/simulator/scenes/meshes/ and use meshFromBlob in robosub_source.zig.
;

pub fn main(init: std.process.Init) !void {
    const allocator = init.arena.allocator();
    const io = init.io;
    const args = try init.minimal.args.toSlice(allocator);
    if (args.len < 2) {
        std.log.err("{s}", .{help});
        return error.Usage;
    }
    var object_name: ?[]const u8 = null;
    var output_path: ?[]const u8 = null;
    var path: ?[]const u8 = null;
    var i: usize = 1;
    while (i < args.len) : (i += 1) {
        const arg = args[i];
        if (std.mem.eql(u8, arg, "--object")) {
            i += 1;
            if (i >= args.len) return error.Usage;
            object_name = args[i];
        } else if (std.mem.eql(u8, arg, "-o") or std.mem.eql(u8, arg, "--output")) {
            i += 1;
            if (i >= args.len) return error.Usage;
            output_path = args[i];
        } else if (path == null) {
            path = arg;
        } else {
            std.log.err("unknown argument: {s}", .{arg});
            return error.Usage;
        }
    }
    const blend_path = path orelse return error.Usage;
    const obj = object_name orelse return error.Usage;
    const out = output_path orelse return error.Usage;

    var file = try blend.BlendFile.load(allocator, io, blend_path);
    defer file.deinit();
    var mesh = try file.exportMeshNamed(obj);
    defer file.deinitMeshExport(mesh);
    const bytes = try mesh_wire.encode(allocator, mesh.positions, mesh.triangles);
    defer allocator.free(bytes);
    const dir = std.Io.Dir.cwd();
    try dir.writeFile(io, .{ .sub_path = out, .data = bytes });
    std.log.info("wrote {s} ({d} verts, {d} tris)", .{ out, mesh.positions.len, mesh.triangles.len });
}