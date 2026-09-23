#pragma once

#include "audio_core/engine.hpp"
#include "audio_core/protocol/command_packet.hpp"
#include "audio_core/protocol/telemetry_packet.hpp"
#include "audio_core/serialization/session_serializer.hpp"

#include <string>
#include <string_view>
#include <vector>
#include <unordered_map>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <iostream>
#include <cstring>
#include <cstdint>

#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

namespace audio_core::network {

namespace detail {

// ============================================================================
// Zero-Dependency RFC 3174 SHA-1 & RFC 4648 Base-64 for RFC 6455 Handshake
// ============================================================================
inline std::string compute_websocket_accept(std::string_view client_key) {
    const std::string magic_guid = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    std::string combined = std::string(client_key) + magic_guid;

    uint32_t h0 = 0x67452301;
    uint32_t h1 = 0xEFCDAB89;
    uint32_t h2 = 0x98BADCFE;
    uint32_t h3 = 0x10325476;
    uint32_t h4 = 0xC3D2E1F0;

    const uint64_t msg_len_bits = combined.size() * 8;
    std::vector<uint8_t> msg(combined.begin(), combined.end());
    msg.push_back(0x80);
    while ((msg.size() % 64) != 56) {
        msg.push_back(0x00);
    }
    for (int i = 7; i >= 0; --i) {
        msg.push_back(static_cast<uint8_t>((msg_len_bits >> (i * 8)) & 0xFF));
    }

    auto rol = [](uint32_t val, uint32_t bits) -> uint32_t {
        return (val << bits) | (val >> (32 - bits));
    };

    for (size_t chunk = 0; chunk < msg.size(); chunk += 64) {
        uint32_t w[80];
        for (int i = 0; i < 16; ++i) {
            w[i] = (static_cast<uint32_t>(msg[chunk + i * 4]) << 24) |
                   (static_cast<uint32_t>(msg[chunk + i * 4 + 1]) << 16) |
                   (static_cast<uint32_t>(msg[chunk + i * 4 + 2]) << 8) |
                   (static_cast<uint32_t>(msg[chunk + i * 4 + 3]));
        }
        for (int i = 16; i < 80; ++i) {
            w[i] = rol(w[i - 3] ^ w[i - 8] ^ w[i - 14] ^ w[i - 16], 1);
        }

        uint32_t a = h0, b = h1, c = h2, d = h3, e = h4;
        for (int i = 0; i < 80; ++i) {
            uint32_t f, k;
            if (i < 20) {
                f = (b & c) | ((~b) & d);
                k = 0x5A827999;
            } else if (i < 40) {
                f = b ^ c ^ d;
                k = 0x6ED9EBA1;
            } else if (i < 60) {
                f = (b & c) | (b & d) | (c & d);
                k = 0x8F1BBCDC;
            } else {
                f = b ^ c ^ d;
                k = 0xCA62C1D6;
            }
            uint32_t temp = rol(a, 5) + f + e + k + w[i];
            e = d;
            d = c;
            c = rol(b, 30);
            b = a;
            a = temp;
        }

        h0 += a;
        h1 += b;
        h2 += c;
        h3 += d;
        h4 += e;
    }

    uint8_t hash[20];
    uint32_t h[5] = {h0, h1, h2, h3, h4};
    for (int i = 0; i < 5; ++i) {
        hash[i * 4]     = static_cast<uint8_t>((h[i] >> 24) & 0xFF);
        hash[i * 4 + 1] = static_cast<uint8_t>((h[i] >> 16) & 0xFF);
        hash[i * 4 + 2] = static_cast<uint8_t>((h[i] >> 8) & 0xFF);
        hash[i * 4 + 3] = static_cast<uint8_t>(h[i] & 0xFF);
    }

    static const char b64[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::string out;
    for (size_t i = 0; i < 20; i += 3) {
        uint32_t n = (static_cast<uint32_t>(hash[i]) << 16) |
                     (static_cast<uint32_t>(i + 1 < 20 ? hash[i + 1] : 0) << 8) |
                     (static_cast<uint32_t>(i + 2 < 20 ? hash[i + 2] : 0));
        out.push_back(b64[(n >> 18) & 63]);
        out.push_back(b64[(n >> 12) & 63]);
        out.push_back(i + 1 < 20 ? b64[(n >> 6) & 63] : '=');
        out.push_back(i + 2 < 20 ? b64[n & 63] : '=');
    }
    return out;
}

// Built-in Embedded WebMixer HTML5 Single-Page Application
inline const char* get_embedded_webmixer_html() {
    return R"rawhtml(<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>AETHEL // SOVEREIGN WEBMIXER</title>
  <style>
    :root {
      --bg: #090b10;
      --panel: #11141c;
      --border: #1e2433;
      --accent: #00ffaa;
      --text: #c0c6d4;
      --mute: #ff3366;
      --solo: #ffcc00;
    }
    * { box-sizing: border-box; margin: 0; padding: 0; font-family: monospace, sans-serif; }
    body { background: var(--bg); color: var(--text); padding: 16px; overflow-x: auto; }
    header { display: flex; align-items: center; justify-content: space-between; border-bottom: 2px solid var(--border); padding-bottom: 12px; margin-bottom: 16px; }
    .title { font-size: 16px; font-weight: bold; color: var(--accent); letter-spacing: 2px; }
    .status-badge { font-size: 12px; padding: 4px 8px; border-radius: 4px; background: #222; border: 1px solid var(--border); }
    .status-badge.online { background: rgba(0,255,170,0.15); color: var(--accent); border-color: var(--accent); }
    .transport-bar { display: flex; gap: 8px; align-items: center; margin-bottom: 16px; background: var(--panel); padding: 8px 16px; border: 1px solid var(--border); border-radius: 4px; }
    button { background: #1a202c; color: var(--text); border: 1px solid var(--border); padding: 6px 12px; cursor: pointer; border-radius: 3px; font-weight: bold; }
    button:hover { background: #2d3748; }
    button.active { background: var(--accent); color: #000; }
    button.btn-mute.active { background: var(--mute); color: #fff; }
    button.btn-solo.active { background: var(--solo); color: #000; }
    .mixer-desk { display: flex; gap: 12px; align-items: flex-start; }
    .channel-strip { width: 110px; background: var(--panel); border: 1px solid var(--border); border-radius: 4px; padding: 10px; display: flex; flex-direction: column; align-items: center; gap: 8px; }
    .channel-strip.master { border-color: var(--accent); }
    .strip-name { font-size: 11px; font-weight: bold; text-align: center; white-space: nowrap; overflow: hidden; text-overflow: ellipsis; width: 100%; }
    .meter-container { display: flex; gap: 4px; height: 160px; background: #05070a; padding: 3px; border-radius: 2px; width: 36px; border: 1px solid #1a202c; }
    .meter-bar { width: 12px; height: 100%; background: #111; position: relative; overflow: hidden; }
    .meter-fill { position: absolute; bottom: 0; left: 0; right: 0; background: linear-gradient(to top, #00ffaa 60%, #ffcc00 85%, #ff3366 100%); transition: height 0.05s ease-out; }
    .fader-wrap { width: 100%; }
    input[type=range] { -webkit-appearance: none; width: 100%; background: #1a202c; height: 6px; border-radius: 3px; outline: none; }
    input[type=range]::-webkit-slider-thumb { -webkit-appearance: none; width: 16px; height: 16px; border-radius: 50%; background: var(--accent); cursor: pointer; }
    .btn-row { display: flex; gap: 4px; width: 100%; justify-content: center; }
    .kinetic-panel { margin-top: 16px; background: var(--panel); border: 1px solid var(--border); padding: 12px; border-radius: 4px; display: flex; gap: 24px; align-items: center; }
    .kinetic-metric { display: flex; flex-direction: column; gap: 4px; }
    .kinetic-val { font-size: 14px; font-weight: bold; }
  </style>
</head>
<body>
  <header>
    <div class="title">⚡ AETHEL SOVEREIGN // REAL-TIME WEBMIXER</div>
    <div id="wsStatus" class="status-badge">WS: CONNECTING...</div>
  </header>

  <div class="transport-bar">
    <button id="btnPlay" onclick="cmdTransport('transport_play')">▶ PLAY</button>
    <button id="btnPause" onclick="cmdTransport('transport_pause')">⏸ PAUSE</button>
    <button id="btnStop" onclick="cmdTransport('transport_stop')">⏹ STOP</button>
    <span style="margin-left: 16px;">TEMPO: <strong id="valBpm">120.0</strong> BPM</span>
    <span style="margin-left: 16px;">POS: <strong id="valPos">0</strong> SMP</span>
    <span style="margin-left: 16px;">CPU: <strong id="valCpu">0.0</strong>%</span>
  </div>

  <div class="mixer-desk" id="mixerDesk">
    <!-- Channel Strips Dynamically Injected -->
  </div>

  <div class="kinetic-panel">
    <strong>KINETIC DYNAMICS:</strong>
    <div class="kinetic-metric"><span>AUTHORITY (SUB):</span><span class="kinetic-val" id="kinAuth" style="color:#00ffaa">0.00</span></div>
    <div class="kinetic-metric"><span>POWER (MIDS):</span><span class="kinetic-val" id="kinPower" style="color:#3399ff">0.00</span></div>
    <div class="kinetic-metric"><span>DETAIL (SLEW):</span><span class="kinetic-val" id="kinDetail" style="color:#ff3366">0.00</span></div>
    <div class="kinetic-metric"><span>CREST FACTOR:</span><span class="kinetic-val" id="kinCrest">0.0 dB</span></div>
  </div>

  <script>
    let ws = null;
    let deskEl = document.getElementById('mixerDesk');
    let statusEl = document.getElementById('wsStatus');

    function connectWS() {
      const proto = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
      const wsUrl = proto + '//' + window.location.host;
      ws = new WebSocket(wsUrl);

      ws.onopen = () => {
        statusEl.className = 'status-badge online';
        statusEl.textContent = 'WS: CONNECTED [30Hz RT]';
      };

      ws.onmessage = (evt) => {
        try {
          const msg = JSON.parse(evt.data);
          if (msg.type === 'telemetry') updateTelemetry(msg);
        } catch (e) {
          console.error(e);
        }
      };

      ws.onclose = () => {
        statusEl.className = 'status-badge';
        statusEl.textContent = 'WS: OFFLINE (RECONNECTING...)';
        setTimeout(connectWS, 1500);
      };
    }

    function sendCmd(obj) {
      if (ws && ws.readyState === WebSocket.OPEN) {
        ws.send(JSON.stringify(obj));
      }
    }

    function cmdTransport(type) {
      sendCmd({ type: type });
    }

    function updateTelemetry(t) {
      document.getElementById('valBpm').textContent = t.transport.bpm.toFixed(1);
      document.getElementById('valPos').textContent = t.transport.sample_position;
      document.getElementById('valCpu').textContent = t.cpu_load.toFixed(1);

      document.getElementById('kinAuth').textContent = t.kinetic.authority.toFixed(3);
      document.getElementById('kinPower').textContent = t.kinetic.power.toFixed(3);
      document.getElementById('kinDetail').textContent = t.kinetic.detail.toFixed(3);
      document.getElementById('kinCrest').textContent = t.kinetic.crest_factor_db.toFixed(1) + ' dB';

      // Render or update master strip
      ensureStrip('master', 0, 'MASTER BUS', t.master);

      // Render or update tracks
      if (t.tracks) {
        t.tracks.forEach(trk => {
          ensureStrip('track', trk.id, trk.name || ('Track ' + trk.id), trk);
        });
      }
    }

    function ensureStrip(type, id, name, data) {
      const elId = 'strip_' + type + '_' + id;
      let strip = document.getElementById(elId);
      if (!strip) {
        strip = document.createElement('div');
        strip.id = elId;
        strip.className = 'channel-strip' + (type === 'master' ? ' master' : '');
        strip.innerHTML = `
          <div class="strip-name">${name}</div>
          <div class="meter-container">
            <div class="meter-bar"><div class="meter-fill" id="${elId}_ml"></div></div>
            <div class="meter-bar"><div class="meter-fill" id="${elId}_mr"></div></div>
          </div>
          <div class="fader-wrap">
            <input type="range" id="${elId}_fader" min="0" max="1.5" step="0.01" value="${data.gain !== undefined ? data.gain : 0.8}"
                   oninput="onFader('${type}', ${id}, this.value)">
          </div>
          ${type !== 'master' ? `
          <div class="btn-row">
            <button class="btn-mute" id="${elId}_mute" onclick="toggleMute(${id}, this)">M</button>
            <button class="btn-solo" id="${elId}_solo" onclick="toggleSolo(${id}, this)">S</button>
          </div>` : `
          <div class="btn-row">
            <button id="${elId}_limiter" class="active" onclick="toggleLimiter(this)">LIMIT</button>
          </div>`}
        `;
        deskEl.appendChild(strip);
      }

      // Update meters
      const ml = document.getElementById(elId + '_ml');
      const mr = document.getElementById(elId + '_mr');
      if (ml) ml.style.height = Math.min(100, (data.peak_l || 0) * 100) + '%';
      if (mr) mr.style.height = Math.min(100, (data.peak_r || 0) * 100) + '%';
    }

    function onFader(type, id, val) {
      if (type === 'master') {
        sendCmd({ type: 'set_master_gain', gain: parseFloat(val) });
      } else {
        sendCmd({ type: 'set_track_gain', track_id: id, gain: parseFloat(val) });
      }
    }

    function toggleMute(id, btn) {
      const active = !btn.classList.contains('active');
      btn.classList.toggle('active', active);
      sendCmd({ type: 'set_track_mute', track_id: id, mute: active });
    }

    function toggleSolo(id, btn) {
      const active = !btn.classList.contains('active');
      btn.classList.toggle('active', active);
      sendCmd({ type: 'set_track_solo', track_id: id, solo: active });
    }

    function toggleLimiter(btn) {
      const active = !btn.classList.contains('active');
      btn.classList.toggle('active', active);
      sendCmd({ type: 'set_master_limiter', enabled: active });
    }

    connectWS();
  </script>
</body>
</html>
)rawhtml";
}

} // namespace detail

// ============================================================================
struct WebSocketConfig {
    uint16_t port{8088};
    std::string host{"0.0.0.0"};
    uint32_t telemetry_rate_hz{30};
    bool serve_embedded_gui{true};
};

// ============================================================================
// WebSocketBridge: High-Performance Real-Time WebMixer Remote Control Bridge
// RFC 6455 Compliant, Non-blocking Lock-Free SPSC Dispatch to AudioEngineCore
// ============================================================================
class WebSocketBridge {
public:
    using Config = WebSocketConfig;

    explicit WebSocketBridge(Engine& engine, const Config& config = {})
        : m_engine(engine),
          m_config(config) {}

    ~WebSocketBridge() {
        stop();
    }

    bool start() {
        if (m_running.load(std::memory_order_relaxed)) return true;

        m_server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (m_server_fd < 0) {
            std::cerr << "[WebSocketBridge] Failed to create socket" << std::endl;
            return false;
        }

        int opt = 1;
        setsockopt(m_server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        // Set non-blocking
        int flags = fcntl(m_server_fd, F_GETFL, 0);
        fcntl(m_server_fd, F_SETFL, flags | O_NONBLOCK);

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(m_config.port);
        inet_pton(AF_INET, m_config.host.c_str(), &addr.sin_addr);

        if (bind(m_server_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            std::cerr << "[WebSocketBridge] Bind failed on " << m_config.host << ":" << m_config.port << std::endl;
            close(m_server_fd);
            m_server_fd = -1;
            return false;
        }

        if (listen(m_server_fd, 16) < 0) {
            std::cerr << "[WebSocketBridge] Listen failed" << std::endl;
            close(m_server_fd);
            m_server_fd = -1;
            return false;
        }

        m_running.store(true, std::memory_order_release);
        m_worker_thread = std::thread(&WebSocketBridge::worker_loop, this);

        std::cout << "[WebSocketBridge] Listening on ws://" << m_config.host << ":" << m_config.port << std::endl;
        return true;
    }

    void stop() {
        if (!m_running.load(std::memory_order_relaxed)) return;

        m_running.store(false, std::memory_order_release);
        if (m_worker_thread.joinable()) {
            m_worker_thread.join();
        }

        std::lock_guard<std::mutex> lock(m_clients_mutex);
        for (auto& [fd, client] : m_clients) {
            close(fd);
        }
        m_clients.clear();

        if (m_server_fd >= 0) {
            close(m_server_fd);
            m_server_fd = -1;
        }
    }

    [[nodiscard]] bool is_running() const noexcept {
        return m_running.load(std::memory_order_relaxed);
    }

    [[nodiscard]] uint16_t port() const noexcept {
        return m_config.port;
    }

    [[nodiscard]] size_t connected_clients() const noexcept {
        std::lock_guard<std::mutex> lock(m_clients_mutex);
        return m_clients.size();
    }

    // Direct Broadcast
    void broadcast_text(std::string_view message) {
        std::lock_guard<std::mutex> lock(m_clients_mutex);
        for (auto& [fd, client] : m_clients) {
            if (client.is_websocket) {
                send_ws_frame(fd, 0x1, message.data(), message.size());
            }
        }
    }

    void broadcast_binary(const void* data, size_t size) {
        std::lock_guard<std::mutex> lock(m_clients_mutex);
        for (auto& [fd, client] : m_clients) {
            if (client.is_websocket) {
                send_ws_frame(fd, 0x2, data, size);
            }
        }
    }

private:
    struct ClientState {
        int fd{-1};
        bool is_websocket{false};
        bool expects_binary{false};
        std::vector<uint8_t> rx_buffer{};
    };

    Engine& m_engine;
    Config m_config;
    std::atomic<bool> m_running{false};
    int m_server_fd{-1};
    std::thread m_worker_thread;

    mutable std::mutex m_clients_mutex;
    std::unordered_map<int, ClientState> m_clients;

    void worker_loop() {
        const auto period = std::chrono::milliseconds(1000 / std::max(1u, m_config.telemetry_rate_hz));
        auto last_telemetry = std::chrono::steady_clock::now();

        std::vector<pollfd> poll_fds;

        while (m_running.load(std::memory_order_relaxed)) {
            poll_fds.clear();

            // 1. Server listener
            poll_fds.push_back({m_server_fd, POLLIN, 0});

            // 2. Client sockets
            {
                std::lock_guard<std::mutex> lock(m_clients_mutex);
                for (const auto& [fd, client] : m_clients) {
                    poll_fds.push_back({fd, POLLIN, 0});
                }
            }

            int ret = poll(poll_fds.data(), poll_fds.size(), 10); // 10ms timeout
            if (ret > 0) {
                // Check new incoming connections
                if (poll_fds[0].revents & POLLIN) {
                    accept_new_clients();
                }

                // Check active clients
                std::vector<int> fds_to_close;
                for (size_t i = 1; i < poll_fds.size(); ++i) {
                    if (poll_fds[i].revents & (POLLIN | POLLERR | POLLHUP | POLLNVAL)) {
                        int client_fd = poll_fds[i].fd;
                        if (!process_client_read(client_fd)) {
                            fds_to_close.push_back(client_fd);
                        }
                    }
                }

                if (!fds_to_close.empty()) {
                    std::lock_guard<std::mutex> lock(m_clients_mutex);
                    for (int fd : fds_to_close) {
                        close(fd);
                        m_clients.erase(fd);
                    }
                }
            }

            // Periodic Telemetry Streaming
            auto now = std::chrono::steady_clock::now();
            if (now - last_telemetry >= period) {
                last_telemetry = now;
                broadcast_telemetry_snapshot();
            }
        }
    }

    void accept_new_clients() {
        while (true) {
            sockaddr_in client_addr{};
            socklen_t client_len = sizeof(client_addr);
            int client_fd = accept(m_server_fd, reinterpret_cast<sockaddr*>(&client_addr), &client_len);
            if (client_fd < 0) {
                break; // No more pending connections
            }

            int flags = fcntl(client_fd, F_GETFL, 0);
            fcntl(client_fd, F_SETFL, flags | O_NONBLOCK);

            int nodelay = 1;
            setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

            std::lock_guard<std::mutex> lock(m_clients_mutex);
            m_clients[client_fd] = ClientState{client_fd, false, false, {}};
        }
    }

    bool process_client_read(int fd) {
        uint8_t buffer[4096];
        ssize_t bytes_read = recv(fd, buffer, sizeof(buffer), 0);
        if (bytes_read <= 0) {
            return false; // Disconnected
        }

        std::lock_guard<std::mutex> lock(m_clients_mutex);
        auto it = m_clients.find(fd);
        if (it == m_clients.end()) return false;

        auto& client = it->second;
        client.rx_buffer.insert(client.rx_buffer.end(), buffer, buffer + bytes_read);

        if (!client.is_websocket) {
            return handle_http_request(client);
        } else {
            return handle_websocket_frames(client);
        }
    }

    bool handle_http_request(ClientState& client) {
        std::string req(client.rx_buffer.begin(), client.rx_buffer.end());
        size_t header_end = req.find("\r\n\r\n");
        if (header_end == std::string::npos) {
            return true; // Wait for full HTTP headers
        }

        // Check if WebSocket Upgrade request
        size_t key_pos = req.find("Sec-WebSocket-Key: ");
        if (key_pos != std::string::npos) {
            key_pos += 19;
            size_t key_end = req.find("\r\n", key_pos);
            if (key_end != std::string::npos) {
                std::string client_key = req.substr(key_pos, key_end - key_pos);
                std::string accept_key = detail::compute_websocket_accept(client_key);

                std::string response =
                    "HTTP/1.1 101 Switching Protocols\r\n"
                    "Upgrade: websocket\r\n"
                    "Connection: Upgrade\r\n"
                    "Sec-WebSocket-Accept: " + accept_key + "\r\n\r\n";

                send(client.fd, response.data(), response.size(), 0);
                client.is_websocket = true;
                client.rx_buffer.erase(client.rx_buffer.begin(), client.rx_buffer.begin() + header_end + 4);
                return true;
            }
        }

        // Fallback: Regular HTTP GET or HEAD -> Serve embedded WebMixer HTML GUI
        if (m_config.serve_embedded_gui && (req.rfind("GET /", 0) == 0 || req.rfind("HEAD /", 0) == 0)) {
            bool is_head = (req.rfind("HEAD /", 0) == 0);
            const char* html = detail::get_embedded_webmixer_html();
            size_t html_len = std::strlen(html);
            std::string response =
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/html; charset=utf-8\r\n"
                "Content-Length: " + std::to_string(html_len) + "\r\n"
                "Connection: close\r\n\r\n";
            if (!is_head) {
                response += html;
            }

            send(client.fd, response.data(), response.size(), 0);
            return false; // Close HTTP 1.0 connection cleanly after serving page
        }

        // Unknown HTTP request -> 404
        std::string not_found = "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\n\r\n";
        send(client.fd, not_found.data(), not_found.size(), 0);
        return false;
    }

    bool handle_websocket_frames(ClientState& client) {
        while (client.rx_buffer.size() >= 2) {
            uint8_t b0 = client.rx_buffer[0];
            uint8_t b1 = client.rx_buffer[1];

            uint8_t opcode = b0 & 0x0F;
            bool masked = (b1 & 0x80) != 0;
            uint64_t payload_len = b1 & 0x7F;

            size_t header_size = 2;
            if (payload_len == 126) {
                if (client.rx_buffer.size() < 4) return true;
                payload_len = (static_cast<uint64_t>(client.rx_buffer[2]) << 8) | client.rx_buffer[3];
                header_size = 4;
            } else if (payload_len == 127) {
                if (client.rx_buffer.size() < 10) return true;
                payload_len = 0;
                for (int i = 0; i < 8; ++i) {
                    payload_len = (payload_len << 8) | client.rx_buffer[2 + i];
                }
                header_size = 10;
            }

            size_t mask_size = masked ? 4 : 0;
            if (client.rx_buffer.size() < header_size + mask_size + payload_len) {
                return true; // Incomplete frame, wait for more bytes
            }

            uint8_t mask_key[4] = {0, 0, 0, 0};
            if (masked) {
                std::memcpy(mask_key, client.rx_buffer.data() + header_size, 4);
            }

            uint8_t* payload_start = client.rx_buffer.data() + header_size + mask_size;
            if (masked) {
                for (uint64_t i = 0; i < payload_len; ++i) {
                    payload_start[i] ^= mask_key[i % 4];
                }
            }

            // Dispatch frame
            if (opcode == 0x1) {
                // Text Frame (JSON command)
                std::string_view json_text(reinterpret_cast<const char*>(payload_start), payload_len);
                dispatch_json_command(client, json_text);
            } else if (opcode == 0x2) {
                // Binary Frame
                dispatch_binary_command(client, payload_start, payload_len);
            } else if (opcode == 0x8) {
                // Close frame
                send_ws_frame(client.fd, 0x8, nullptr, 0);
                return false;
            } else if (opcode == 0x9) {
                // Ping -> Reply Pong
                send_ws_frame(client.fd, 0xA, payload_start, payload_len);
            }

            size_t total_frame_len = header_size + mask_size + payload_len;
            client.rx_buffer.erase(client.rx_buffer.begin(), client.rx_buffer.begin() + total_frame_len);
        }
        return true;
    }

    void send_ws_frame(int fd, uint8_t opcode, const void* data, size_t size) {
        std::vector<uint8_t> frame;
        frame.push_back(0x80 | (opcode & 0x0F)); // FIN = 1

        if (size <= 125) {
            frame.push_back(static_cast<uint8_t>(size));
        } else if (size <= 65535) {
            frame.push_back(126);
            frame.push_back(static_cast<uint8_t>((size >> 8) & 0xFF));
            frame.push_back(static_cast<uint8_t>(size & 0xFF));
        } else {
            frame.push_back(127);
            for (int i = 7; i >= 0; --i) {
                frame.push_back(static_cast<uint8_t>((size >> (i * 8)) & 0xFF));
            }
        }

        if (data && size > 0) {
            const auto* bytes = static_cast<const uint8_t*>(data);
            frame.insert(frame.end(), bytes, bytes + size);
        }

        send(fd, frame.data(), frame.size(), MSG_NOSIGNAL);
    }

    void dispatch_json_command(ClientState& client, std::string_view json_str) {
        using namespace serialization::json;
        auto val_opt = Parser::parse(json_str);
        if (!val_opt || !val_opt->is_object()) return;

        const auto& obj = *val_opt;
        const auto* type_val = obj.get("type");
        if (!type_val || !type_val->is_string()) return;

        const std::string& type = type_val->as_string();

        if (type == "set_track_gain") {
            uint32_t trk = obj.get("track_id") ? obj.get("track_id")->as_uint(1) : 1;
            float gain = obj.get("gain") ? obj.get("gain")->as_float(1.0f) : 1.0f;
            protocol::MixerCommand cmd{};
            cmd.type = protocol::MixerCommandType::SetTrackGain;
            cmd.target_id = trk;
            cmd.value1 = gain;
            m_engine.send_command(cmd);

        } else if (type == "set_track_pan") {
            uint32_t trk = obj.get("track_id") ? obj.get("track_id")->as_uint(1) : 1;
            float pan = obj.get("pan") ? obj.get("pan")->as_float(0.0f) : 0.0f;
            protocol::MixerCommand cmd{};
            cmd.type = protocol::MixerCommandType::SetTrackPan;
            cmd.target_id = trk;
            cmd.value1 = pan;
            m_engine.send_command(cmd);

        } else if (type == "set_track_mute") {
            uint32_t trk = obj.get("track_id") ? obj.get("track_id")->as_uint(1) : 1;
            bool mute = obj.get("mute") ? obj.get("mute")->as_bool(false) : false;
            protocol::MixerCommand cmd{};
            cmd.type = protocol::MixerCommandType::SetTrackMute;
            cmd.target_id = trk;
            cmd.flags = mute ? 1 : 0;
            m_engine.send_command(cmd);

        } else if (type == "set_track_solo") {
            uint32_t trk = obj.get("track_id") ? obj.get("track_id")->as_uint(1) : 1;
            bool solo = obj.get("solo") ? obj.get("solo")->as_bool(false) : false;
            protocol::MixerCommand cmd{};
            cmd.type = protocol::MixerCommandType::SetTrackSolo;
            cmd.target_id = trk;
            cmd.flags = solo ? 1 : 0;
            m_engine.send_command(cmd);

        } else if (type == "set_master_gain") {
            float gain = obj.get("gain") ? obj.get("gain")->as_float(1.0f) : 1.0f;
            protocol::MixerCommand cmd{};
            cmd.type = protocol::MixerCommandType::SetMasterGain;
            cmd.value1 = gain;
            m_engine.send_command(cmd);

        } else if (type == "set_master_limiter") {
            bool enabled = obj.get("enabled") ? obj.get("enabled")->as_bool(true) : true;
            protocol::MixerCommand cmd{};
            cmd.type = protocol::MixerCommandType::SetMasterLimiter;
            cmd.flags = enabled ? 1 : 0;
            m_engine.send_command(cmd);

        } else if (type == "set_bus_gain") {
            uint32_t bus_id = obj.get("bus_id") ? obj.get("bus_id")->as_uint(1) : 1;
            float gain = obj.get("gain") ? obj.get("gain")->as_float(1.0f) : 1.0f;
            protocol::MixerCommand cmd{};
            cmd.type = protocol::MixerCommandType::SetBusGain;
            cmd.target_id = bus_id;
            cmd.value1 = gain;
            m_engine.send_command(cmd);

        } else if (type == "set_slot_param") {
            uint32_t trk = obj.get("track_id") ? obj.get("track_id")->as_uint(1) : 1;
            uint32_t slot = obj.get("slot_id") ? obj.get("slot_id")->as_uint(0) : 0;
            uint32_t param = obj.get("param_id") ? obj.get("param_id")->as_uint(0) : 0;
            float val = obj.get("value") ? obj.get("value")->as_float(0.0f) : 0.0f;
            protocol::MixerCommand cmd{};
            cmd.type = protocol::MixerCommandType::SetTrackSlotParam;
            cmd.target_id = trk;
            cmd.secondary_id = (slot << 16) | (param & 0xFFFF);
            cmd.value1 = val;
            m_engine.send_command(cmd);

        } else if (type == "set_slot_bypass") {
            uint32_t trk = obj.get("track_id") ? obj.get("track_id")->as_uint(1) : 1;
            uint32_t slot = obj.get("slot_id") ? obj.get("slot_id")->as_uint(0) : 0;
            bool bypass = obj.get("bypass") ? obj.get("bypass")->as_bool(false) : false;
            protocol::MixerCommand cmd{};
            cmd.type = protocol::MixerCommandType::SetTrackSlotBypass;
            cmd.target_id = trk;
            cmd.secondary_id = slot;
            cmd.flags = bypass ? 1 : 0;
            m_engine.send_command(cmd);

        } else if (type == "transport_play") {
            m_engine.transport_play();

        } else if (type == "transport_pause") {
            m_engine.transport_pause();

        } else if (type == "transport_stop") {
            m_engine.transport_stop();

        } else if (type == "transport_seek") {
            double beat = obj.get("beat") ? obj.get("beat")->as_double(0.0) : 0.0;
            m_engine.transport_seek(beat);

        } else if (type == "set_tempo") {
            double bpm = obj.get("bpm") ? obj.get("bpm")->as_double(120.0) : 120.0;
            m_engine.set_tempo(bpm);

        } else if (type == "note_on") {
            uint8_t note = obj.get("note") ? static_cast<uint8_t>(obj.get("note")->as_uint(60)) : 60;
            uint8_t vel = obj.get("velocity") ? static_cast<uint8_t>(obj.get("velocity")->as_uint(100)) : 100;
            m_engine.note_on(note, vel);

        } else if (type == "note_off") {
            uint8_t note = obj.get("note") ? static_cast<uint8_t>(obj.get("note")->as_uint(60)) : 60;
            m_engine.note_off(note);

        } else if (type == "all_notes_off") {
            m_engine.all_notes_off();

        } else if (type == "launch_clip") {
            uint32_t trk = obj.get("track_id") ? obj.get("track_id")->as_uint(1) : 1;
            uint32_t clip = obj.get("clip_id") ? obj.get("clip_id")->as_uint(0) : 0;
            protocol::MixerCommand cmd{};
            cmd.type = protocol::MixerCommandType::LaunchTrackClip;
            cmd.target_id = trk;
            cmd.secondary_id = clip;
            m_engine.send_command(cmd);

        } else if (type == "stop_clip") {
            uint32_t trk = obj.get("track_id") ? obj.get("track_id")->as_uint(1) : 1;
            protocol::MixerCommand cmd{};
            cmd.type = protocol::MixerCommandType::StopTrackClip;
            cmd.target_id = trk;
            m_engine.send_command(cmd);

        } else if (type == "launch_scene") {
            uint32_t scene = obj.get("scene_id") ? obj.get("scene_id")->as_uint(0) : 0;
            protocol::MixerCommand cmd{};
            cmd.type = protocol::MixerCommandType::LaunchScene;
            cmd.target_id = scene;
            m_engine.send_command(cmd);

        } else if (type == "stop_all_clips") {
            protocol::MixerCommand cmd{};
            cmd.type = protocol::MixerCommandType::StopAllClips;
            m_engine.send_command(cmd);

        } else if (type == "subscribe") {
            const auto* fmt = obj.get("format");
            if (fmt && fmt->as_string() == "binary") {
                client.expects_binary = true;
            } else {
                client.expects_binary = false;
            }

        } else if (type == "get_session") {
            serialization::ProjectSessionData sess = serialization::SessionSerializer::extract_session(m_engine.mixer(), m_engine.clock(), "Web Session");
            std::string resp = "{\"type\":\"session_state\",\"data\":" + sess.to_json() + "}";
            send_ws_frame(client.fd, 0x1, resp.data(), resp.size());
        }
    }

    void dispatch_binary_command(ClientState& client, const uint8_t* data, size_t size) {
        if (size == sizeof(protocol::MixerCommand)) {
            protocol::MixerCommand cmd{};
            std::memcpy(&cmd, data, sizeof(cmd));
            m_engine.send_command(cmd);
        }
    }

    void broadcast_telemetry_snapshot() {
        std::lock_guard<std::mutex> lock(m_clients_mutex);
        if (m_clients.empty()) return;

        protocol::MixerTelemetryFrame telemetry{};
        m_engine.mixer().capture_telemetry_snapshot(telemetry);

        const auto pos = m_engine.clock().position_snapshot();
        const float cpu = m_engine.cpu_load_percent();

        // 1. Build JSON telemetry string
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3);
        ss << "{\"type\":\"telemetry\","
           << "\"render_cycle\":" << telemetry.render_cycle << ","
           << "\"timestamp_ns\":" << telemetry.timestamp_ns << ","
           << "\"active_tracks\":" << telemetry.active_tracks << ","
           << "\"active_buses\":" << telemetry.active_buses << ","
           << "\"cpu_load\":" << cpu << ","
           << "\"transport\":{"
           << "\"playing\":" << (pos.is_playing ? "true" : "false") << ","
           << "\"bpm\":" << pos.bpm << ","
           << "\"sample_position\":" << pos.sample_position << ","
           << "\"beat\":" << pos.total_beats
           << "},"
           << "\"master\":{"
           << "\"peak_l\":" << telemetry.master_meter.peak_l << ","
           << "\"peak_r\":" << telemetry.master_meter.peak_r << ","
           << "\"rms_l\":" << telemetry.master_meter.rms_l << ","
           << "\"rms_r\":" << telemetry.master_meter.rms_r
           << "},"
           << "\"kinetic\":{"
           << "\"authority\":" << telemetry.kinetic_meter.authority << ","
           << "\"power\":" << telemetry.kinetic_meter.power << ","
           << "\"detail\":" << telemetry.kinetic_meter.detail << ","
           << "\"crest_factor_db\":" << telemetry.kinetic_meter.crest_factor_db << ","
           << "\"diagnostic_id\":" << telemetry.kinetic_meter.diagnostic_id
           << "},";

        // Tracks array
        ss << "\"tracks\":[";
        uint32_t active_count = 0;
        for (size_t i = 1; i <= MixerGraph::kMaxTracks; ++i) {
            auto* trk = m_engine.mixer().get_track(static_cast<uint32_t>(i));
            if (trk && trk->is_active()) {
                if (active_count > 0) ss << ",";
                auto tm = trk->meter();
                ss << "{\"id\":" << trk->id() << ","
                   << "\"name\":\"" << trk->name() << "\","
                   << "\"gain\":" << trk->gain() << ","
                   << "\"pan\":" << trk->pan() << ","
                   << "\"mute\":" << (trk->is_muted() ? "true" : "false") << ","
                   << "\"solo\":" << (trk->is_solo() ? "true" : "false") << ","
                   << "\"peak_l\":" << tm.peak_l << ","
                   << "\"peak_r\":" << tm.peak_r << ","
                   << "\"rms_l\":" << tm.rms_l << ","
                   << "\"rms_r\":" << tm.rms_r << "}";
                active_count++;
            }
        }
        ss << "]}";

        std::string json_str = ss.str();

        // Broadcast to clients
        for (auto& [fd, client] : m_clients) {
            if (!client.is_websocket) continue;

            if (client.expects_binary) {
                send_ws_frame(fd, 0x2, &telemetry, sizeof(telemetry));
            } else {
                send_ws_frame(fd, 0x1, json_str.data(), json_str.size());
            }
        }
    }
};

} // namespace audio_core::network
