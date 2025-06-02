// main_server.cpp
#include <iostream>
#include <thread>
#include <vector>
#include <cstring>
#include <stdexcept>

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

#include "SIL.hpp"

constexpr int PORT = 5555;
constexpr int WAYPOINT_SIZE = sizeof(Waypoint);
constexpr int FRAME_SIZE = sizeof(Frame);

// Add this constant
constexpr size_t MAX_SOCKET_CHUNK = 10 * 1024 * 1024; // 10MB per syscall

// Update helpers
bool recv_all(socket_t sock, char* buffer, size_t length) {
    size_t total_received = 0;
    
    while (total_received < length) {
        size_t remaining = length - total_received;
        size_t chunk_size = std::min(remaining, MAX_SOCKET_CHUNK);

        #ifdef _WIN32
            int to_receive = static_cast<int>(std::min(chunk_size, 
                static_cast<size_t>(std::numeric_limits<int>::max())));
            int received = recv(sock, buffer + total_received, to_receive, 0);
        #else
            ssize_t received = recv(sock, buffer + total_received, chunk_size, 0);
        #endif
        
        if (received <= 0) return false;
        total_received += static_cast<size_t>(received);
    }
    return true;
}

bool send_all(socket_t sock, const char* buffer, size_t length) {
    size_t total_sent = 0;
    
    while (total_sent < length) {
        size_t remaining = length - total_sent;
        size_t chunk_size = std::min(remaining, MAX_SOCKET_CHUNK);

        #ifdef _WIN32
            int to_send = static_cast<int>(std::min(chunk_size, 
                static_cast<size_t>(std::numeric_limits<int>::max())));
            int sent = send(sock, buffer + total_sent, to_send, 0);
        #else
            ssize_t sent = send(sock, buffer + total_sent, chunk_size, 0);
        #endif
        
        if (sent <= 0) return false;
        total_sent += static_cast<size_t>(sent);
    }
    return true;
}

// Update handle_client function
void handle_client(socket_t sock) {
    try {
        // Read number of waypoints (4 bytes)
        int32_t n;
        if (!recv_all(sock, reinterpret_cast<char*>(&n), sizeof(n))) {
            throw std::runtime_error("Failed to read waypoint count");
        }

        // Validate trajectory size
        if (n < 0) {
            throw std::runtime_error("Invalid negative waypoint count");
        }
        if (static_cast<size_t>(n) > MAX_TRAJECTORY_POINTS) {
            throw std::runtime_error("Waypoint count exceeds maximum limit");
        }

        // Read elbow position and arm parameters (3+2 doubles)
        double elbow_x, elbow_y, elbow_z, l_arm_proth;
        if (!recv_all(sock, reinterpret_cast<char*>(&elbow_x), sizeof(double))) throw std::runtime_error("Failed to read Elbow x");
        if (!recv_all(sock, reinterpret_cast<char*>(&elbow_y), sizeof(double))) throw std::runtime_error("Failed to read Elbow y");
        if (!recv_all(sock, reinterpret_cast<char*>(&elbow_z), sizeof(double))) throw std::runtime_error("Failed to read Elbow z");
        if (!recv_all(sock, reinterpret_cast<char*>(&l_arm_proth), sizeof(double))) throw std::runtime_error("Failed to read Elbow l_arm_proth");
        
        Eigen::Vector3d elbow_pos(elbow_x, elbow_y, elbow_z);

        // Read waypoints in chunks for large trajectories
        std::vector<Waypoint> trajectory;
        trajectory.reserve(n);
        
        constexpr size_t CHUNK_SIZE = 1000;
        size_t points_remaining = n;
        
        while (points_remaining > 0) {
            size_t chunk_points = std::min(points_remaining, CHUNK_SIZE);
            size_t chunk_bytes = chunk_points * WAYPOINT_SIZE;
            
            // Temporary storage for chunk
            std::vector<Waypoint> chunk(chunk_points);
            
            if (!recv_all(sock, reinterpret_cast<char*>(chunk.data()), chunk_bytes)) {
                throw std::runtime_error("Incomplete waypoint data");
            }
            
            // Move into main trajectory
            trajectory.insert(trajectory.end(), 
                             std::make_move_iterator(chunk.begin()),
                             std::make_move_iterator(chunk.end()));
            
            points_remaining -= chunk_points;
            
            // Progress reporting
            std::cout << "Received: " << (n - points_remaining) << "/" << n
                      << " waypoints (" << (100 * (n - points_remaining) / n) << "%)"
                      << std::endl;
        }

        // Run simulation - now capturing ideal_torques
        std::cout << "Starting simulation for " << n << " waypoints..." << std::endl;
        std::vector<Frame> results;
        std::vector<IdealTorquePoint> ideal_torques;
        run_sil_simulation(trajectory, results, ideal_torques, elbow_pos, l_arm_proth);
        std::cout << "Simulation completed with " << results.size() << " frames and "
                  << ideal_torques.size() << " ideal torque points" << std::endl;

        // Check frame count
        if (results.size() > MAX_FRAME_POINTS) {
            throw std::runtime_error("Frame count exceeds maximum limit");
        }

        // Send header: frame count (m) + ideal torque count (k)
        int32_t m = static_cast<int32_t>(results.size());
        int32_t k = static_cast<int32_t>(ideal_torques.size());
        int32_t header[2] = {m, k}; // Combined header

        if (!send_all(sock, reinterpret_cast<const char*>(header), sizeof(header))) {
            throw std::runtime_error("Failed to send header");
        }

        // Send frames in chunks
        if (m > 0) {
            constexpr size_t FRAME_CHUNK_SIZE = 500;
            size_t frames_remaining = m;
            size_t frames_sent = 0;
            
            while (frames_remaining > 0) {
                size_t chunk_frames = std::min(frames_remaining, FRAME_CHUNK_SIZE);
                size_t chunk_bytes = chunk_frames * FRAME_SIZE;
                size_t offset = frames_sent;
                
                if (!send_all(sock, 
                            reinterpret_cast<const char*>(results.data() + offset),
                            chunk_bytes)) {
                    throw std::runtime_error("Incomplete frame data sent");
                }
                
                frames_remaining -= chunk_frames;
                frames_sent += chunk_frames;
                
                // Progress reporting
                std::cout << "Sent: " << frames_sent << "/" << m
                        << " frames (" << (100 * frames_sent / m) << "%)"
                        << std::endl;
            }
        }

        // NEW: Send ideal torques in chunks
        if (k > 0) {
            constexpr size_t IDEAL_CHUNK_SIZE = 500;
            size_t points_remaining = k;
            size_t points_sent = 0;
            
            while (points_remaining > 0) {
                size_t chunk_points = std::min(points_remaining, IDEAL_CHUNK_SIZE);
                size_t chunk_bytes = chunk_points * sizeof(IdealTorquePoint);
                size_t offset = points_sent;
                
                if (!send_all(sock, 
                            reinterpret_cast<const char*>(ideal_torques.data() + offset),
                            chunk_bytes)) {
                    throw std::runtime_error("Incomplete ideal torque data sent");
                }
                
                points_remaining -= chunk_points;
                points_sent += chunk_points;
                
                // Progress reporting
                std::cout << "Sent: " << points_sent << "/" << k
                        << " ideal torques (" << (100 * points_sent / k) << "%)"
                        << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] " << e.what() << std::endl;
    }

    // Close socket
}

int main() {
    #ifdef _WIN32
        // Initialize Winsock
        WSADATA wsaData;
        int result = WSAStartup(MAKEWORD(2, 2), &wsaData);
        if (result != 0) {
            std::cerr << "WSAStartup failed: " << result << std::endl;
            return 1;
        }
    #endif

    socket_t server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd == INVALID_SOCKET_VALUE) {
        std::cerr << "Socket creation failed" << std::endl;
        #ifdef _WIN32
            WSACleanup();
        #endif
        return 1;
    }

    // Reuse address option
    int opt = 1;
    #ifdef _WIN32
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, 
                      reinterpret_cast<const char*>(&opt), sizeof(opt)) == SOCKET_ERROR_VALUE) {
    #else
        if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
    #endif
        std::cerr << "setsockopt failed" << std::endl;
        #ifdef _WIN32
            closesocket(server_fd);
            WSACleanup();
        #else
            close(server_fd);
        #endif
        return 1;
    }

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(PORT);

    if (bind(server_fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) < 0) {
        std::cerr << "Bind failed" << std::endl;
        #ifdef _WIN32
            closesocket(server_fd);
            WSACleanup();
        #else
            close(server_fd);
        #endif
        return 1;
    }

    if (listen(server_fd, 10) < 0) {
        std::cerr << "Listen failed" << std::endl;
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
        sockaddr_in client_addr;
        #ifdef _WIN32
            int client_addr_len = sizeof(client_addr);
        #else
            socklen_t client_addr_len = sizeof(client_addr);
        #endif
        
        socket_t client_sock = accept(server_fd, 
                                     reinterpret_cast<sockaddr*>(&client_addr), 
                                     &client_addr_len);
        if (client_sock == INVALID_SOCKET_VALUE) {
            std::cerr << "Accept failed" << std::endl;
            continue;
        }

        // Print client info
        char client_ip[INET_ADDRSTRLEN];
        inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, INET_ADDRSTRLEN);
        std::cout << "Accepted connection from " << client_ip << ":" 
                  << ntohs(client_addr.sin_port) << std::endl;

        std::thread(handle_client, client_sock).detach();
    }

    // Cleanup (unreachable in infinite loop, but for completeness)
    #ifdef _WIN32
        closesocket(server_fd);
        WSACleanup();
    #else
        close(server_fd);
    #endif

    return 0;
}