#include "storm_teleop/spark_status_reader.hpp"

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

namespace storm_teleop
{

namespace
{

// Same layout as sparkcan's CreateArbId:
// DeviceType(5) | Mfr(8) | APIClass(6) | APIIndex(4) | DeviceID(6)
constexpr uint32_t kDeviceType = 0x02;
constexpr uint32_t kManufacturer = 0x05;
constexpr uint32_t kApiPeriod0 = (6 << 4) | 0;
constexpr uint32_t kApiPeriod1 = (6 << 4) | 1;
constexpr uint32_t kApiPeriod2 = (6 << 4) | 2;

constexpr float kVoltageScale = 1 << 7;
constexpr float kCurrentScale = 1 << 5;

uint32_t arb_id(uint32_t api, uint8_t device_id)
{
  return (kDeviceType << 24) | (kManufacturer << 16) | (api << 6) | device_id;
}

template<typename T>
T unpack(const uint8_t * d, int offset = 0)
{
  T v;
  std::memcpy(&v, d + offset, sizeof(T));
  return v;
}

}  // namespace

SparkStatusReader::~SparkStatusReader()
{
  close();
}

void SparkStatusReader::open(const std::string & iface, const std::vector<uint8_t> & device_ids)
{
  close();

  soc_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (soc_ < 0) {
    throw std::runtime_error(std::string("status socket: ") + std::strerror(errno));
  }

  std::vector<can_filter> filters;
  for (uint8_t id : device_ids) {
    for (uint32_t api : {kApiPeriod0, kApiPeriod1, kApiPeriod2}) {
      filters.push_back({arb_id(api, id) | CAN_EFF_FLAG, CAN_EFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG});
    }
    status_[id] = SparkStatus{};
  }
  ::setsockopt(soc_, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
    static_cast<socklen_t>(filters.size() * sizeof(can_filter)));

  ifreq ifr{};
  std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
  if (::ioctl(soc_, SIOCGIFINDEX, &ifr) < 0) {
    close();
    throw std::runtime_error("status socket: no interface '" + iface + "'");
  }

  sockaddr_can addr{};
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;
  if (::bind(soc_, reinterpret_cast<sockaddr *>(&addr), sizeof(addr)) < 0) {
    close();
    throw std::runtime_error(std::string("status socket bind: ") + std::strerror(errno));
  }

  running_ = true;
  thread_ = std::thread(&SparkStatusReader::run, this);
}

void SparkStatusReader::close()
{
  running_ = false;
  if (thread_.joinable()) thread_.join();
  if (soc_ >= 0) {
    ::close(soc_);
    soc_ = -1;
  }
}

SparkStatus SparkStatusReader::get(uint8_t device_id) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto it = status_.find(device_id);
  return it == status_.end() ? SparkStatus{} : it->second;
}

void SparkStatusReader::run()
{
  pollfd pfd{soc_, POLLIN, 0};
  can_frame frame{};

  while (running_) {
    if (::poll(&pfd, 1, 50) <= 0) continue;
    if (::read(soc_, &frame, sizeof(frame)) != sizeof(frame)) continue;

    const uint32_t id = frame.can_id & CAN_EFF_MASK;
    const uint8_t device_id = id & 0x3F;
    const uint32_t api = (id >> 6) & 0x3FF;
    const uint8_t * d = frame.data;

    std::lock_guard<std::mutex> lock(mutex_);
    auto it = status_.find(device_id);
    if (it == status_.end()) continue;
    SparkStatus & s = it->second;

    if (api == kApiPeriod0) {
      s.sticky_faults = unpack<uint16_t>(d, 4);
    } else if (api == kApiPeriod1) {
      s.velocity_rpm = unpack<float>(d);
      s.temperature_c = d[4];
      s.voltage = (unpack<uint16_t>(d, 5) & 0x0FFF) / kVoltageScale;
      s.current_a = (unpack<uint16_t>(d, 6) >> 4) / kCurrentScale;
      s.last_status1 = std::chrono::steady_clock::now();
      s.seen = true;
    } else if (api == kApiPeriod2) {
      s.position_rot = unpack<float>(d);
    }
  }
}

}  // namespace storm_teleop
