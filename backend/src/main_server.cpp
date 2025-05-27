#include <iostream>
#include <thread>
#include <vector>
#include <boost/asio.hpp>
#include <nlohmann/json.hpp>
#include "SIL.hpp"

using json = nlohmann::json;
namespace asio = boost::asio;
using asio::ip::tcp;

constexpr int PORT = 5555;

void handle_client(tcp::socket socket) {
    try {
        // Read data
        asio::streambuf buffer;
        asio::read_until(socket, buffer, "\n");
        std::string data(asio::buffers_begin(buffer.data()), asio::buffers_end(buffer.data()));
        data.erase(std::remove(data.begin(), data.end(), '\n'), data.end());

        // Parse JSON
        json full_json = json::parse(data);
        if (!full_json.contains("trajectory") || !full_json["trajectory"].is_array()) {
            throw std::runtime_error("Malformed input: expected key 'trajectory'");
        }

        Trajectory traj = parseTrajectoryFromJson(full_json.at("trajectory"));

        // Run simulation
        std::vector<json> all_results;
        run_sil_simulation(traj, all_results);

        // Send response
        std::string response = json(all_results).dump() + "\n";
        asio::write(socket, asio::buffer(response));

    } catch (const std::exception& e) {
        std::cerr << "[Server] Error handling client: " << e.what() << std::endl;
    }
}


int main() {
    try {
        asio::io_context io_context;
        tcp::acceptor acceptor(io_context, tcp::endpoint(tcp::v4(), PORT));
        
        std::cout << "[main_server] Listening on port " << PORT << std::endl;

        while (true) {
            tcp::socket socket = acceptor.accept();
            std::thread(handle_client, std::move(socket)).detach();
        }
    } catch (const std::exception& e) {
        std::cerr << "Server error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}