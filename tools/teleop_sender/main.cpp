#include <SDL.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <cerrno>
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

// Encode six IEEE 754 floats as 24 little-endian bytes,
// matching the Zig decoder.
std::array<std::uint8_t, 24> encodeGoal(const std::array<float, 6>& goal) {
    std::array<std::uint8_t, 24> packet{};

    for (std::size_t i = 0; i < goal.size(); ++i) {
        std::uint32_t bits;
        std::memcpy(&bits, &goal[i], sizeof(bits));

        packet[i * 4 + 0] = static_cast<std::uint8_t>(bits);
        packet[i * 4 + 1] = static_cast<std::uint8_t>(bits >> 8);
        packet[i * 4 + 2] = static_cast<std::uint8_t>(bits >> 16);
        packet[i * 4 + 3] = static_cast<std::uint8_t>(bits >> 24);
    }

    return packet;
}

int main(int argc, char** argv) {
    // Goal order: x, y, z, roll, pitch, yaw
    std::array<float, 6> goal{0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f};
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
    constexpr float step = 0.05f;
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

    std::cout << "Focus the Teleop window to control the goal. Close it to exit.\n";

    while (running) {
        SDL_Event event;

        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                running = false;
            } else if (event.type == send_event_type) {
                const Uint8* keys = SDL_GetKeyboardState(nullptr);

                // Focus the Teleop window and hold keys to move its absolute
                // world-frame goal. Each 50 ms timer tick changes a value by
                // 0.05: W/S = +X/-X, D/A = +Y/-Y, R/F = +Z/-Z.
                // E/Q = +roll/-roll, Up/Down = +pitch/-pitch, and
                // Right/Left = +yaw/-yaw. Angles are in radians.
                if (keys[SDL_SCANCODE_W]) goal[0] += step;
                if (keys[SDL_SCANCODE_S]) goal[0] -= step;
                if (keys[SDL_SCANCODE_D]) goal[1] += step;
                if (keys[SDL_SCANCODE_A]) goal[1] -= step;
                if (keys[SDL_SCANCODE_R]) goal[2] += step;
                if (keys[SDL_SCANCODE_F]) goal[2] -= step;

                // Orientation: roll, pitch, yaw
                if (keys[SDL_SCANCODE_E])     goal[3] += step;
                if (keys[SDL_SCANCODE_Q])     goal[3] -= step;
                if (keys[SDL_SCANCODE_UP])    goal[4] += step;
                if (keys[SDL_SCANCODE_DOWN])  goal[4] -= step;
                if (keys[SDL_SCANCODE_RIGHT]) goal[5] += step;
                if (keys[SDL_SCANCODE_LEFT])  goal[5] -= step;

                const auto packet = encodeGoal(goal);
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
