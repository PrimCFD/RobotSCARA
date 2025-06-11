#include <iostream>
#include <thread>
#include <vector>
#include <cstring>
#include <stdexcept>
#include <limits>           // for std::numeric_limits

#ifdef __linux__
  #include <signal.h>
#endif

#include "SIL.hpp"
#include "SocketUtils.hpp"

void handle_client(socket_t sock) {
    try {
        // On Linux, ignore SIGPIPE so writing to a closed socket returns EPIPE
  #ifdef __linux__
        signal(SIGPIPE, SIG_IGN);
  #endif

        // Read waypoint count
        int32_t n = 0;
        if (!recv_all(sock, reinterpret_cast<char*>(&n), sizeof(n)))
            throw std::runtime_error("Failed to read waypoint count");
        if (n < 0 || static_cast<size_t>(n) > MAX_TRAJECTORY_POINTS)
            throw std::runtime_error("Invalid waypoint count");

        // Read elbow position + arm parameter
        double ex, ey, ez, l_arm;
        if (!recv_all(sock, reinterpret_cast<char*>(&ex), sizeof(ex)))
            throw std::runtime_error("Failed to read elbow X");
        if (!recv_all(sock, reinterpret_cast<char*>(&ey), sizeof(ey)))
            throw std::runtime_error("Failed to read elbow Y");
        if (!recv_all(sock, reinterpret_cast<char*>(&ez), sizeof(ez)))
            throw std::runtime_error("Failed to read elbow Z");
        if (!recv_all(sock, reinterpret_cast<char*>(&l_arm), sizeof(l_arm)))
            throw std::runtime_error("Failed to read arm length");
        Eigen::Vector3d elbow_pos(ex, ey, ez);

        // Read waypoints in chunks
        std::vector<Waypoint> trajectory;
        trajectory.reserve(n);
        constexpr size_t CHUNK_POINTS = 1000;
        size_t remaining = n;
        while (remaining > 0) {
            size_t this_chunk = std::min(remaining, CHUNK_POINTS);
            std::vector<Waypoint> tmp(this_chunk);
            if (!recv_all(sock,
                          reinterpret_cast<char*>(tmp.data()),
                          this_chunk * WAYPOINT_SIZE))
                throw std::runtime_error("Incomplete waypoint data");
            trajectory.insert(trajectory.end(),
                              std::make_move_iterator(tmp.begin()),
                              std::make_move_iterator(tmp.end()));
            remaining -= this_chunk;
        }

        // Send header: [ frameCount (s) | waypointCount ]
        double end_time    = trajectory.back().t;
        int32_t waypointCount = static_cast<int32_t>(trajectory.size());

        if (!send_all(sock,
                    reinterpret_cast<const char*>(&end_time),
                    sizeof(end_time)))
            throw std::runtime_error("Failed to send header (end_time)");

        if (!send_all(sock,
                    reinterpret_cast<const char*>(&waypointCount),
                    sizeof(waypointCount)))
        throw std::runtime_error("Failed to send header (waypointCount)");

        // Stream simulation frames and ideal torques
        run_sil_streaming(trajectory, sock, elbow_pos, l_arm);

        // Half-close write end so client sees EOF
  #ifdef _WIN32
        shutdown(sock, SD_SEND);
  #else
        shutdown(sock, SHUT_WR);
  #endif

    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << std::endl;
    }

    // Clean up socket
  #ifdef _WIN32
    closesocket(sock);
  #else
    close(sock);
  #endif
}

int main() {
  #ifdef _WIN32
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2,2), &wsa) != 0) {
        std::cerr << "WSAStartup failed\n";
        return 1;
    }
  #endif

    socket_t server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == INVALID_SOCKET_VALUE) {
        std::cerr << "Socket creation failed\n";
  #ifdef _WIN32
        WSACleanup();
  #endif
        return 1;
    }

    int opt = 1;
  #ifdef _WIN32
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR,
                   reinterpret_cast<const char*>(&opt), sizeof(opt))
        == SOCKET_ERROR_VALUE) {
  #else
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
  #endif
        std::cerr << "setsockopt failed\n";
  #ifdef _WIN32
        closesocket(server_fd);
        WSACleanup();
  #else
        close(server_fd);
  #endif
        return 1;
    }

    sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port        = htons(PORT);

    if (bind(server_fd,
             reinterpret_cast<sockaddr*>(&addr),
             sizeof(addr)) < 0) {
        std::cerr << "Bind failed\n";
  #ifdef _WIN32
        closesocket(server_fd);
        WSACleanup();
  #else
        close(server_fd);
  #endif
        return 1;
    }

    if (listen(server_fd, 10) < 0) {
        std::cerr << "Listen failed\n";
  #ifdef _WIN32
        closesocket(server_fd);
        WSACleanup();
  #else
        close(server_fd);
  #endif
        return 1;
    }

    std::cout << "[Server] Listening on port " << PORT << std::endl;

    while (true) {
        sockaddr_in client{};
  #ifdef _WIN32
        int len = sizeof(client);
  #else
        socklen_t len = sizeof(client);
  #endif
        socket_t client_sock = accept(server_fd,
                                      reinterpret_cast<sockaddr*>(&client),
                                      &len);
        if (client_sock == INVALID_SOCKET_VALUE) {
            std::cerr << "Accept failed\n";
            continue;
        }

        char ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET,
                  &client.sin_addr,
                  ip,
                  sizeof(ip));
        std::cout << "Accepted connection from "
                  << ip << ':' << ntohs(client.sin_port)
                  << std::endl;

        std::thread(handle_client, client_sock).detach();
    }

    // (Not reached)
  #ifdef _WIN32
    closesocket(server_fd);
    WSACleanup();
  #else
    close(server_fd);
  #endif
    return 0;
}
