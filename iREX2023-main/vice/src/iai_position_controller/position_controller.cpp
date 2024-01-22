#include "EIPScanner/cip/connectionManager/NetworkConnectionParams.h"

#include "vice/iai_position_controller/position_controller.h"

using namespace eipScanner::cip;
using eipScanner::cip::connectionManager::ConnectionParameters;
using eipScanner::cip::connectionManager::NetworkConnectionParams;
using eipScanner::utils::Logger;

using namespace iai_position_controller;

PositionController::PositionController(
    const ConnectionPath connection_path,
    const uint rpi,
    const DataByteSize data_size,
    const LogLevel log_level
  )
{
  Logger::setLogLevel(log_level);
  active_ = -1;

  connection_parameters_.connectionPath = std::vector<uint8_t>(
    std::begin(connection_paths_[connection_path]),
    std::end(connection_paths_[connection_path])
    );
  connection_parameters_.o2tRealTimeFormat = true;
  connection_parameters_.originatorVendorId = 699;
  connection_parameters_.originatorSerialNumber = 0x12345;
  connection_parameters_.t2oNetworkConnectionParams |= NetworkConnectionParams::P2P;
  connection_parameters_.t2oNetworkConnectionParams |= NetworkConnectionParams::SCHEDULED_PRIORITY;
  connection_parameters_.t2oNetworkConnectionParams |= data_size;
  connection_parameters_.o2tNetworkConnectionParams |= NetworkConnectionParams::P2P;
  connection_parameters_.o2tNetworkConnectionParams |= NetworkConnectionParams::SCHEDULED_PRIORITY;
  connection_parameters_.o2tNetworkConnectionParams |= data_size;
  connection_parameters_.o2tRPI = rpi;
  connection_parameters_.t2oRPI = rpi;
  connection_parameters_.transportTypeTrigger |= NetworkConnectionParams::CLASS1;

  data_size_ = data_size;
}

PositionController::~PositionController()
{
  active_ = -1;
  cleanup();
}

void PositionController::cleanup()
{
  for (auto slave = slaves_.begin(); slave != slaves_.end(); slave++) {
    connection_manager_.forwardClose(slave->first, slave->second);
  }
}

std::shared_ptr<SlaveUnit> PositionController::register_slave_unit(
    const std::string& device_ip,
    const uint port
    )
{
  try {
    auto si = std::make_shared<SessionInfo>(device_ip, port);
    auto io = connection_manager_.forwardOpen(si, connection_parameters_);
    auto slave = std::make_shared<SlaveUnit>(io, data_size_);
    slaves_.insert(std::make_pair(si, io));
    active_ = 100;
    return slave;
  } catch (std::exception& e) {
    Logger(LogLevel::ERROR) << "can not register session";
  }
  return nullptr;
}

bool PositionController::execute()
{
  if (active_ <= 0) {
    Logger(LogLevel::ERROR) << "not active";
    return false;
  }
  controller_th_ = std::make_shared<std::thread>(
    [&]() {
      while (active_ > 0) {
        connection_manager_.handleConnections(std::chrono::milliseconds(active_));
      }
    });
  return true;
}

void PositionController::wait()
{
  controller_th_->join();
}

