#include "SocketUtils.hpp"
#include <algorithm>        // for std::min


bool recv_all(socket_t sock, char* buffer, size_t length) {
    size_t total_received = 0;
    while (total_received < length) {
        size_t remaining  = length - total_received;
        size_t chunk_sz  = std::min(remaining, MAX_SOCKET_CHUNK);
    #ifdef _WIN32
        int to_recv = static_cast<int>(std::min(chunk_sz,
                              static_cast<size_t>(std::numeric_limits<int>::max())));
        int recvd   = recv(sock, buffer + total_received, to_recv, 0);
    #else
        ssize_t recvd = recv(sock, buffer + total_received, chunk_sz, 0);
    #endif
        if (recvd <= 0) return false;
        total_received += static_cast<size_t>(recvd);
    }
    return true;
}

bool send_all(socket_t sock, const char* buffer, size_t length) {
    size_t total_sent = 0;
    while (total_sent < length) {
        size_t remaining = length - total_sent;
        size_t chunk_sz  = std::min(remaining, MAX_SOCKET_CHUNK);
    #ifdef _WIN32
        int to_send = static_cast<int>(std::min(chunk_sz,
                              static_cast<size_t>(std::numeric_limits<int>::max())));
        int sent    = send(sock, buffer + total_sent, to_send, 0);
    #else
        ssize_t sent = send(sock, buffer + total_sent, chunk_sz, 0);
    #endif
        if (sent <= 0) return false;
        total_sent += static_cast<size_t>(sent);
    }
    return true;
}