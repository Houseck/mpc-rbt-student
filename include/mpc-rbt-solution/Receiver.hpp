#ifndef COMMUNICATION_EXAMPLE_RECEIVER_H
#define COMMUNICATION_EXAMPLE_RECEIVER_H

#include <mpc-rbt-solution/util/Socket.hpp>
#include <mpc-rbt-solution/util/Utils.hpp>

namespace Receiver
{
class Node : public Socket::UDP
{
public:
  explicit Node(const Utils::Config::Receiver & receiverConfig)
  : Socket::UDP(receiverConfig.localPort), config(receiverConfig)
  {
    // 1. Inicializace callbacku (propojení s metodou onDataReceived)
    callback = std::bind(&Node::onDataReceived, this, std::placeholders::_1);

    // 2. Příprava socketu (pokud by se metody v Socket.hpp jmenovaly jinak, jen je tady přepíšeš)
    create();
    configure();
    bind();
  }

  void run();

private:
  void onDataReceived(const Socket::IPFrame & frame);

  Utils::Message data{};

  Utils::Config::Receiver config;
  std::function<void(const Socket::IPFrame &)> callback;
  rclcpp::Logger logger{rclcpp::get_logger("receiver")};

  friend class TestMock;
};
}  // namespace Receiver

namespace Receiver
{
class TestMock : public Node
{
public:
  TestMock(const Utils::Config::Receiver & receiverConfig) : Receiver::Node(receiverConfig) {}

  [[nodiscard]] const Utils::Message & getData() const { return data; }
};
}  // namespace Receiver

#endif  // COMMUNICATION_EXAMPLE_RECEIVER_H
