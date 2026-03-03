#include <mpc-rbt-solution/Sender.hpp>

void Sender::Node::run()
{
  while (errno != EINTR) {
    if ((std::chrono::steady_clock::now() - timer_tick) < timer_period) continue;
    timer_tick = std::chrono::steady_clock::now();

    callback();
  }
}

void Sender::Node::onDataTimerTick()
{
  data.x += 1.5;
  data.y += 1.0;
  data.z += 0.5;

  data.timestamp =
    static_cast<uint64_t>(std::chrono::system_clock::now().time_since_epoch().count());

  Socket::IPFrame frame{
    .port = config.remotePort,
    .address = config.remoteAddress,
  };
  /*
  nlohmann::json j;
  Utils::Message::to_json(j, data);
  std::string json_str = j.dump();
  
  std::copy(json_str.begin(), json_str.end(), frame.serializedData.begin());
  frame.dataSize = json_str.size();
  */
  Utils::Message::serialize(frame, data);

  if (!send(frame)) {
    RCLCPP_ERROR(logger, "Failed to send data to network!");
  }

  RCLCPP_INFO(logger, "Sending data to host: '%s:%d'", frame.address.c_str(), frame.port);

  RCLCPP_INFO(logger, "\n\tstamp: %ld\n\tx: %f\n\ty: %f\n\tz: %f", 
              data.timestamp, data.x, data.y, data.z);
}
