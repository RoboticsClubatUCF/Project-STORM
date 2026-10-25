#ifndef STORM_TELEOP__SPARK_STATUS_READER_HPP_
#define STORM_TELEOP__SPARK_STATUS_READER_HPP_

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace storm_teleop
{

struct SparkStatus
{
  float velocity_rpm{0.0f};
  float position_rot{0.0f};
  float voltage{0.0f};
  float temperature_c{0.0f};
  float current_a{0.0f};
  uint16_t sticky_faults{0};
  bool seen{false};
  std::chrono::steady_clock::time_point last_status1{};
};

/// Listen-only decoder for SPARK MAX periodic status frames.
///
/// sparkcan shares one socket across all SparkMax instances and every
/// instance's thread reads from it, so each frame lands in a random thread
/// and is dropped if it's for another device. SocketCAN gives each socket
/// its own copy of every frame, so a separate socket here sees all of them.
class SparkStatusReader
{
public:
  ~SparkStatusReader();

  /// Throws std::runtime_error if the socket can't be opened.
  void open(const std::string & iface, const std::vector<uint8_t> & device_ids);
  void close();

  SparkStatus get(uint8_t device_id) const;

private:
  void run();

  int soc_{-1};
  std::atomic_bool running_{false};
  std::thread thread_;
  mutable std::mutex mutex_;
  std::map<uint8_t, SparkStatus> status_;
};

}  // namespace storm_teleop

#endif  // STORM_TELEOP__SPARK_STATUS_READER_HPP_
