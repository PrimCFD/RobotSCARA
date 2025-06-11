#pragma once

#ifdef _WIN32
    #define NOMINMAX
    #include <winsock2.h>
    #include <ws2tcpip.h>
    #pragma comment(lib, "ws2_32.lib")
    using socket_t = SOCKET;
    constexpr socket_t INVALID_SOCKET_VALUE = INVALID_SOCKET;
    constexpr int SOCKET_ERROR_VALUE = SOCKET_ERROR;
#else
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <unistd.h>
    #include <arpa/inet.h>
    using socket_t = int;
    constexpr socket_t INVALID_SOCKET_VALUE = -1;
    constexpr int SOCKET_ERROR_VALUE = -1;
#endif

#include <cstddef>
#include "Types.hpp"

static constexpr int PORT = 5555;
static constexpr size_t WAYPOINT_SIZE = sizeof(Waypoint);
static constexpr size_t FRAME_SIZE    = sizeof(Frame);
static constexpr size_t MAX_SOCKET_CHUNK = 10 * 1024 * 1024; // 10 MB per syscall

bool recv_all(socket_t sock, char* buffer, size_t length);
bool send_all(socket_t sock, const char* buffer, size_t length);