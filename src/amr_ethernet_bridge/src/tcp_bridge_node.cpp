/**
 * tcp_bridge_node.cpp
 *
 * TCP Server running on Raspberry Pi 5.
 * STM32 connects as TCP client to 192.168.0.100:8888
 *
 * STM32 sends plain ASCII strings terminated with '\n', e.g.:
 *   "Hello from client, message 0\n"
 *   "$ACK_ROBOT_ON#\n"
 *
 * This node:
 *   Publishes  → /stm32/rx  (std_msgs/String) : data received FROM STM32
 *   Subscribes → /stm32/tx  (std_msgs/String) : data to send TO STM32
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

#include <string>
#include <thread>
#include <mutex>
#include <atomic>

class TcpBridgeNode : public rclcpp::Node
{
public:
  TcpBridgeNode()
  : Node("tcp_bridge_node"),
    client_fd_(-1),
    connected_(false)
  {
    // ── Parameters ────────────────────────────────────────────────────────
    this->declare_parameter<std::string>("host", "0.0.0.0");
    this->declare_parameter<int>("port", 8888);

    host_ = this->get_parameter("host").as_string();
    port_ = this->get_parameter("port").as_int();

    // ── Publisher: data received FROM STM32 ───────────────────────────────
    pub_rx_ = this->create_publisher<std_msgs::msg::String>("/stm32/rx", 10);

    // ── Subscriber: data to send TO STM32 ────────────────────────────────
    sub_tx_ = this->create_subscription<std_msgs::msg::String>(
      "/stm32/tx", 10,
      std::bind(&TcpBridgeNode::tx_callback, this, std::placeholders::_1));

    // ── Start TCP server in background thread ─────────────────────────────
    server_thread_ = std::thread(&TcpBridgeNode::run_server, this);

    RCLCPP_INFO(this->get_logger(),
      "TCP bridge started — listening on %s:%d", host_.c_str(), port_);
    RCLCPP_INFO(this->get_logger(), "Waiting for STM32 to connect...");
  }

  ~TcpBridgeNode()
  {
    // Signal server thread to stop and close any open socket
    connected_ = false;
    if (client_fd_ >= 0) {
      ::close(client_fd_);
    }
    if (server_thread_.joinable()) {
      server_thread_.join();
    }
  }

private:
  // ── Send a string to STM32 when /stm32/tx receives a message ─────────────
  void tx_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string text = msg->data;
    if (text.empty()) return;

    // Ensure newline terminator — STM32 LwIP recv splits on '\n'
    if (text.back() != '\n') {
      text += '\n';
    }

    std::lock_guard<std::mutex> lock(sock_mutex_);
    if (connected_ && client_fd_ >= 0) {
      ssize_t sent = ::send(client_fd_, text.c_str(), text.size(), MSG_NOSIGNAL);
      if (sent < 0) {
        RCLCPP_WARN(this->get_logger(), "Send failed — STM32 may have disconnected");
        connected_ = false;
      } else {
        RCLCPP_INFO(this->get_logger(),
          "[RPi → STM32]: %s", msg->data.c_str());
      }
    } else {
      RCLCPP_WARN(this->get_logger(),
        "Cannot send — STM32 not connected yet");
    }
  }

  // ── TCP server: creates socket, binds, listens, accepts connections ────────
  void run_server()
  {
    // Create server socket
    int server_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create socket");
      return;
    }

    // Allow reuse so restart doesn't hit "address already in use"
    int opt = 1;
    ::setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // Bind to host:port
    struct sockaddr_in addr{};
    addr.sin_family      = AF_INET;
    addr.sin_port        = htons(static_cast<uint16_t>(port_));
    addr.sin_addr.s_addr = INADDR_ANY;

    if (::bind(server_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(this->get_logger(), "Bind failed on port %d", port_);
      ::close(server_fd);
      return;
    }

    ::listen(server_fd, 1);

    // Accept loop — reconnects automatically if STM32 drops
    while (rclcpp::ok()) {
      struct sockaddr_in client_addr{};
      socklen_t client_len = sizeof(client_addr);

      int fd = ::accept(
        server_fd,
        reinterpret_cast<sockaddr*>(&client_addr),
        &client_len);

      if (fd < 0) {
        RCLCPP_WARN(this->get_logger(), "Accept error — retrying");
        continue;
      }

      RCLCPP_INFO(this->get_logger(),
        "STM32 connected from %s:%d",
        inet_ntoa(client_addr.sin_addr),
        ntohs(client_addr.sin_port));

      {
        std::lock_guard<std::mutex> lock(sock_mutex_);
        client_fd_ = fd;
        connected_ = true;
      }

      // Block here receiving data until connection drops
      receive_loop(fd);

      {
        std::lock_guard<std::mutex> lock(sock_mutex_);
        connected_ = false;
        client_fd_ = -1;
      }

      ::close(fd);
      RCLCPP_INFO(this->get_logger(),
        "STM32 disconnected — waiting for reconnect...");
    }

    ::close(server_fd);
  }

  // ── Receive loop: reads data, splits on newline, publishes each line ───────
  void receive_loop(int fd)
  {
    char    raw[512];
    std::string buf;          // accumulates partial data between recv() calls

    while (rclcpp::ok()) {
      ssize_t n = ::recv(fd, raw, sizeof(raw) - 1, 0);

      if (n <= 0) {
        // n == 0 → clean close by STM32
        // n <  0 → socket error
        break;
      }

      raw[n] = '\0';
      buf   += raw;

      // Extract complete lines (STM32 terminates every send with '\n')
      size_t pos;
      while ((pos = buf.find('\n')) != std::string::npos) {
        std::string line = buf.substr(0, pos);
        buf.erase(0, pos + 1);

        // Trim carriage return if present
        if (!line.empty() && line.back() == '\r') {
          line.pop_back();
        }

        if (line.empty()) continue;

        RCLCPP_INFO(this->get_logger(), "[STM32 → RPi]: %s", line.c_str());

        auto out_msg = std_msgs::msg::String();
        out_msg.data = line;
        pub_rx_->publish(out_msg);
      }
    }
  }

  // ── Member variables ───────────────────────────────────────────────────────
  std::string host_;
  int         port_;

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr    pub_rx_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_tx_;

  std::thread  server_thread_;
  std::mutex   sock_mutex_;

  int                client_fd_;
  std::atomic<bool>  connected_;
};

// ── main ──────────────────────────────────────────────────────────────────────
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TcpBridgeNode>());
  rclcpp::shutdown();
  return 0;
}
