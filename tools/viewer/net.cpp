#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

int udp_open(int port) {
#ifdef _WIN32
  WSADATA wsa;
  if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) return -1;
#endif

  const auto fd = socket(AF_INET, SOCK_DGRAM, 0);

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<unsigned short>(port));
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) return -1;

#ifdef _WIN32
  u_long nonblocking = 1;
  ioctlsocket(fd, FIONBIO, &nonblocking);
#else
  fcntl(fd, F_SETFL, O_NONBLOCK);
#endif

  return static_cast<int>(fd);
}

int udp_send_open(const char *ip, int port) {
  const auto fd = socket(AF_INET, SOCK_DGRAM, 0);
  if (fd < 0) return -1;

  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  addr.sin_port = htons(static_cast<unsigned short>(port));
  if (inet_pton(AF_INET, ip, &addr.sin_addr) != 1 ||
      connect(fd, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) != 0) {
#ifdef _WIN32
    closesocket(fd);
#else
    close(fd);
#endif
    return -1;
  }

  return static_cast<int>(fd);
}

int udp_send(int fd, const void *buf, int size) {
#ifdef _WIN32
  return send(fd, reinterpret_cast<const char *>(buf), size, 0);
#else
  return static_cast<int>(send(fd, buf, static_cast<size_t>(size), 0));
#endif
}

int udp_read(int fd, char *buf, int size) {
  const int n = static_cast<int>(recv(fd, buf, size - 1, 0));
  if (n <= 0) return 0;

  buf[n] = '\0';
  return n;
}
