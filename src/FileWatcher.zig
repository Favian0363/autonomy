const std = @import("std");
const linux = std.os.linux;

const FileWatcher = @This();

fd: std.posix.fd_t,
wd: i32,

dir: []const u8,
name: []const u8,

buf: []u8,
len: usize,
off: usize,

const RawEvent = extern struct {
    wd: i32,
    mask: u32,
    cookie: u32,
    len: u32,
};

const Event = struct {
    raw: RawEvent,
    name: []const u8,
};

pub fn init(arena: std.mem.Allocator, path: []const u8) !FileWatcher {
    const dir = std.fs.path.dirname(path) orelse ".";

    const fd_result = linux.inotify_init1(
        linux.IN.NONBLOCK | linux.IN.CLOEXEC,
    );

    if (linux.errno(fd_result) != .SUCCESS) {
        return error.InotifyInitFailed;
    }

    const fd: std.posix.fd_t = @intCast(fd_result);
    errdefer _ = linux.close(fd);

    return .{
        .fd = fd,
        .wd = try addWatch(fd, dir),
        .dir = dir,
        .name = std.fs.path.basename(path),
        .buf = try arena.alloc(u8, 4096),
        .len = 0,
        .off = 0,
    };
}

pub fn changed(self: *FileWatcher) !bool {
    while (true) {
        if (self.off >= self.len) {
            const result = linux.read(self.fd, self.buf.ptr, self.buf.len);

            if (linux.errno(result) != .SUCCESS) {
                return switch (linux.errno(result)) {
                    .AGAIN, .INTR => false,
                    else => error.InotifyReadFailed,
                };
            }

            if (result == 0) {
                return false;
            }

            self.len = @intCast(result);
            self.off = 0;
        }

        const event = self.nextEvent() orelse {
            return error.InotifyMalformedEvent;
        };

        // std.log.debug(
        //     "watch event: wd={} expected={} mask=0x{x} len={} name={s}",
        //     .{
        //         event.raw.wd,
        //         self.wd,
        //         event.raw.mask,
        //         event.raw.len,
        //         event.name,
        //     },
        // );

        if (event.raw.wd != self.wd) {
            continue;
        }

        if (!std.mem.eql(u8, event.name, self.name)) {
            continue;
        }

        if ((event.raw.mask & linux.IN.MOVED_TO) != 0) {
            return true;
        }

        if ((event.raw.mask & linux.IN.CLOSE_WRITE) != 0) {
            return true;
        }
    }
}

fn nextEvent(self: *FileWatcher) ?Event {
    const header_size = @sizeOf(RawEvent);

    // We don't have enough bytes for an event header.
    if (self.len - self.off < header_size) {
        return null;
    }

    const start = self.off;

    const event_ptr: *align(1) const RawEvent =
        @ptrCast(self.buf[start..].ptr);

    const event = event_ptr.*;

    const total_size =
        header_size + @as(usize, event.len);

    // The event extends beyond the bytes returned by read().
    if (self.len - start < total_size) {
        return null;
    }

    const name_buf =
        self.buf[start + header_size ..][0..event.len];

    const nul =
        std.mem.indexOfScalar(u8, name_buf, 0) orelse name_buf.len;

    const name = name_buf[0..nul];

    self.off += std.mem.alignForward(
        usize,
        total_size,
        @alignOf(RawEvent),
    );

    return .{
        .raw = event,
        .name = name,
    };
}

fn addWatch(fd: std.posix.fd_t, path: []const u8) !i32 {
    var path_buf: [std.fs.max_path_bytes]u8 = undefined;

    const path_z = std.fmt.bufPrintSentinel(
        &path_buf,
        "{s}",
        .{path},
        0,
    ) catch return error.PathTooLong;

    const result = linux.inotify_add_watch(
        fd,
        path_z.ptr,
        linux.IN.CLOSE_WRITE | linux.IN.MOVED_TO,
    );

    if (linux.errno(result) != .SUCCESS) {
        return error.InotifyAddWatchFailed;
    }

    return @intCast(result);
}
