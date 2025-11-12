#include <array>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

#include "daqv2/fsm.hpp"
#include "daqv2/sensors.hpp"
#include "daqv2/states.hpp"
#include "daqv2/telemetry.hpp"
#include "daqv2/time_utils.hpp"
#include "daqv2/udp_io.hpp"

int main(int argc, char** argv) {
    EngineFSM fsm;
    fsm.register_state("Idle", std::make_shared<IdleState>());

    if (argc >= 2 && std::string(argv[1]) == "--server") {
        if (argc < 3) { std::cerr << "Usage: --server <port>\n"; return 1; }
        uint16_t port = (uint16_t)std::stoi(argv[2]);

        UdpRx rx; if (!rx.bind_any(port)) { std::cerr << "Bind failed\n"; return 1; }
        std::array<uint8_t, 2048> buf;
        std::cout << "Server listening on UDP " << port << " ...\n";

        for (;;) {
            auto n = rx.recv(buf.data(), buf.size());
            if (n <= 0) continue;
            PacketHeader hdr; const uint8_t* payload=nullptr;
            if (!parse_and_verify(buf.data(), (size_t)n, hdr, payload)) continue;

            switch (hdr.type) {
              case PKT_HB: {
                const auto* hb = reinterpret_cast<const HeartbeatPayload*>(payload);
                std::cout << "[HB] board=" << int(hdr.board_id)
                          << " seq=" << hdr.seq
                          << " t_us=" << hdr.timestamp_us
                          << " status=" << int(hb->status) << "\n";
              } break;
              case PKT_SENSOR: {
                const auto* sp = reinterpret_cast<const SensorPayload*>(payload);
                std::cout << "[SENS] board=" << int(hdr.board_id)
                          << " PT_HP=" << sp->PT_HP
                          << " PT_LP=" << sp->PT_LP << "\n";
              } break;
            }
        }
    }

    else if (argc >= 2 && std::string(argv[1]) == "--board") {
        if (argc < 4) { std::cerr << "Usage: --board <dst_ip> <port>\n"; return 1; }
        const char* ip = argv[2];
        uint16_t port = (uint16_t)std::stoi(argv[3]);

        UdpTx tx; if (!tx.open(ip, port)) { std::cerr << "Open TX failed\n"; return 1; }
        std::cout << "Board sending to " << ip << ":" << port << " ...\n";

        SystemParameters params{};
        read_sensors_into(params);
        uint32_t seq = 0;
        const double hb_hz = 10.0;
        const double sens_hz = 50.0;
        uint64_t next_hb = now_us(), next_sens = now_us();
        const uint64_t us_per_hb   = (uint64_t)(1e6 / hb_hz);
        const uint64_t us_per_sens = (uint64_t)(1e6 / sens_hz);

        for (;;) {
            uint64_t t = now_us();

            read_sensors_into(params);

            if (t >= next_hb) {
                HeartbeatPayload hb{0};
                auto pkt = make_packet(PKT_HB, /*board_id=*/1, seq++, t, hb);
                tx.send(pkt);
                next_hb += us_per_hb;
            }

            if (t >= next_sens) {
                SensorPayload sp{}; fill_sensor_payload(params, sp);
                auto pkt = make_packet(PKT_SENSOR, /*board_id=*/1, seq++, t, sp);
                tx.send(pkt);
                next_sens += us_per_sens;
            }

            fsm.update(params);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    else {
        std::cout << "Usage:\n  " << argv[0] << " --server <port>\n"
                  << "  " << argv[0] << " --board <dst_ip> <port>\n";
    }
    return 0;
}
