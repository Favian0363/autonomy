#include <SDL.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>

struct TimerData {
    Uint32 event_type;
};

Uint32 onTimer(Uint32 interval_ms, void* userdata) {
    const auto* data = static_cast<TimerData*>(userdata);
    SDL_Event event{};
    event.type = data->event_type;
    SDL_PushEvent(&event);

    return interval_ms; // repeat after this many milliseconds
}

struct Quat {
    float x;
    float y;
    float z;
    float w;
};

Quat quatMul(Quat a, Quat b) {
    return {
        a.w * b.x + b.w * a.x + (a.y * b.z - a.z * b.y),
        a.w * b.y + b.w * a.y + (a.z * b.x - a.x * b.z),
        a.w * b.z + b.w * a.z + (a.x * b.y - a.y * b.x),
        a.w * b.w - (a.x * b.x + a.y * b.y + a.z * b.z),
    };
}

Quat axisAngle(float ax, float ay, float az, float angle) {
    const float half = angle * 0.5f;
    const float s = std::sin(half);
    return {ax * s, ay * s, az * s, std::cos(half)};
}

void normalizeQuat(Quat& q) {
    const float n = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    if (n > 1e-6f) {
        q.x /= n;
        q.y /= n;
        q.z /= n;
        q.w /= n;
    }
}

void normalize3(std::array<float, 3>& v) {
    const float n = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (n > 1e-6f) {
        v[0] /= n;
        v[1] /= n;
        v[2] /= n;
    }
}

// Encode little-endian IEEE 754 floats for the Zig teleop decoder.
// Order: body translation xyz, world orientation quaternion xyzw.
template <std::size_t N>
std::array<std::uint8_t, N * 4> encodeTeleopPacket(const std::array<float, N>& values) {
    std::array<std::uint8_t, N * 4> packet{};

    for (std::size_t i = 0; i < N; ++i) {
        std::uint32_t bits;
        std::memcpy(&bits, &values[i], sizeof(bits));

        packet[i * 4 + 0] = static_cast<std::uint8_t>(bits);
        packet[i * 4 + 1] = static_cast<std::uint8_t>(bits >> 8);
        packet[i * 4 + 2] = static_cast<std::uint8_t>(bits >> 16);
        packet[i * 4 + 3] = static_cast<std::uint8_t>(bits >> 24);
    }

    return packet;
}

int main(int argc, char** argv) {
    // Identity is level, camera along body +Y.
    Quat orientation{0.f, 0.f, 0.f, 1.f};
    unsigned long port = 8080;
    if (argc > 2) {
        std::cerr << "Usage: " << argv[0] << " [udp-port]\n";
        return 2;
    }
    if (argc == 2) {
        char* end = nullptr;
        errno = 0;
        port = std::strtoul(argv[1], &end, 10);
        if (errno != 0 || end == argv[1] || *end != '\0' || port == 0 || port > 65535) {
            std::cerr << "Invalid UDP port: " << argv[1] << '\n';
            return 2;
        }
    }

    int exit_code = 1;
    bool sdl_initialized = false;
    SDL_Window* window = nullptr;
    int sock = -1;
    SDL_TimerID timer = 0;
    Uint32 send_event_type = 0;
    TimerData timer_data{};
    sockaddr_in address{};
    bool running = true;
    constexpr float look_step = 0.05f;
    constexpr Uint32 period_ms = 50;

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << '\n';
        goto cleanup;
    }
    sdl_initialized = true;

    window = SDL_CreateWindow(
        "Teleop",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        400,
        200,
        SDL_WINDOW_SHOWN
    );

    if (window == nullptr) {
        std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << '\n';
        goto cleanup;
    }

    sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::perror("socket");
        goto cleanup;
    }

    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<std::uint16_t>(port));

    if (inet_pton(AF_INET, "127.0.0.1", &address.sin_addr) != 1) {
        std::cerr << "Invalid destination IP address\n";
        goto cleanup;
    }

    send_event_type = SDL_RegisterEvents(1);
    if (send_event_type == static_cast<Uint32>(-1)) {
        std::cerr << "SDL_RegisterEvents failed\n";
        goto cleanup;
    }

    timer_data.event_type = send_event_type;
    timer = SDL_AddTimer(period_ms, onTimer, &timer_data);

    if (timer == 0) {
        std::cerr << "SDL_AddTimer failed: " << SDL_GetError() << '\n';
        goto cleanup;
    }

    std::cout << "Focus the Teleop window. W/S surge, A/D sway, R/F heave.\n"
              << "Orientation: Left/Right yaw (world Z). Up/Down pitch (body X). I/K roll (body Y).\n";

    while (running) {
        SDL_Event event;

        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            } else if (event.type == send_event_type) {
                const Uint8* keys = SDL_GetKeyboardState(nullptr);

                // Body translation while the key is held. Camera forward is body +Y.
                // Z is down, so R dives and F climbs. The mission tracks this as a body error
                // with the translation PID. Releasing the keys commands nothing in translation.
                std::array<float, 3> translation{0.f, 0.f, 0.f};
                if (keys[SDL_SCANCODE_D]) translation[0] += 1.f;
                if (keys[SDL_SCANCODE_A]) translation[0] -= 1.f;
                if (keys[SDL_SCANCODE_W]) translation[1] += 1.f;
                if (keys[SDL_SCANCODE_S]) translation[1] -= 1.f;
                if (keys[SDL_SCANCODE_R]) translation[2] += 1.f;
                if (keys[SDL_SCANCODE_F]) translation[2] -= 1.f;
                normalize3(translation);

                // Yaw about world +Z. Pitch about body X. Roll about body +Y (camera forward).
                if (keys[SDL_SCANCODE_RIGHT]) orientation = quatMul(axisAngle(0.f, 0.f, 1.f, -look_step), orientation);
                if (keys[SDL_SCANCODE_LEFT]) orientation = quatMul(axisAngle(0.f, 0.f, 1.f, look_step), orientation);
                if (keys[SDL_SCANCODE_UP]) orientation = quatMul(orientation, axisAngle(1.f, 0.f, 0.f, -look_step));
                if (keys[SDL_SCANCODE_DOWN]) orientation = quatMul(orientation, axisAngle(1.f, 0.f, 0.f, look_step));
                if (keys[SDL_SCANCODE_I]) orientation = quatMul(orientation, axisAngle(0.f, 1.f, 0.f, look_step));
                if (keys[SDL_SCANCODE_K]) orientation = quatMul(orientation, axisAngle(0.f, 1.f, 0.f, -look_step));
                normalizeQuat(orientation);

                const std::array<float, 7> command{
                    translation[0], translation[1], translation[2],
                    orientation.x, orientation.y, orientation.z, orientation.w,
                };
                const auto packet = encodeTeleopPacket(command);
                const auto sent = sendto(
                    sock,
                    packet.data(),
                    packet.size(),
                    0,
                    reinterpret_cast<const sockaddr*>(&address),
                    sizeof(address)
                );

                if (sent != static_cast<ssize_t>(packet.size())) {
                    std::perror("sendto");
                }
            }
        }

        SDL_Delay(1); // avoid busy-spinning between events
    }

    exit_code = 0;

cleanup:
    if (timer != 0) SDL_RemoveTimer(timer);
    if (sock >= 0) close(sock);
    if (window != nullptr) SDL_DestroyWindow(window);
    if (sdl_initialized) SDL_Quit();
    return exit_code;
}
