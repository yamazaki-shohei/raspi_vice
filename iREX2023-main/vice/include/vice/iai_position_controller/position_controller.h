#ifndef _IAI_POSITION_CONTROLLER_MASTER_UNIT_H_
#define _IAI_POSITION_CONTROLLER_MASTER_UNIT_H_

#include <thread>
#include <string>
#include <map>

#include "EIPScanner/SessionInfo.h"
#include "EIPScanner/IOConnection.h"
#include "EIPScanner/ConnectionManager.h"
#include "EIPScanner/cip/connectionManager/ConnectionParameters.h"
#include "EIPScanner/utils/Logger.h"

#include "vice/iai_position_controller/slave_unit.h"

using eipScanner::SessionInfo;
using eipScanner::IOConnection;
using eipScanner::ConnectionManager;
using eipScanner::cip::connectionManager::ConnectionParameters;
using eipScanner::utils::LogLevel;

namespace iai_position_controller
{

class PositionController
{
private:

  enum DataByteSize : uint8_t
  {
    REMOTE_IO     = 2,
    EASY_POSITION = 8,
    HALF_POSITION = 16,
    FULL_POSITION = 32,
  };

  static constexpr uint8_t connection_paths_[][8] = {
    { 0x20, 0x04, 0x24, 0x05, 0x2C, 0x96, 0x2C, 0x64 }, // exclusive output
    { 0x20, 0x04, 0x24, 0x05, 0x2C, 0x03, 0x2C, 0x64 }, // input only
    { 0x20, 0x04, 0x24, 0x05, 0x2C, 0x04, 0x2C, 0x64 }, // listen only
    { 0x20, 0x04, 0x24, 0x05, 0x2C, 0x06, 0x2C, 0x64 }, // input only ext
    { 0x20, 0x04, 0x24, 0x05, 0x2C, 0x07, 0x2C, 0x64 }, // listen only ext
  };

  enum ConnectionPath : uint8_t
  {
    EXCLUSIVE_OUTPUT = 0,
    INPUT_ONLY,
    LISTEN_ONLY,
    INPUT_ONLY_EXT,
    LISTEN_ONLY_EXT,
  };

public:
  explicit PositionController(
    const ConnectionPath connection_path = ConnectionPath::EXCLUSIVE_OUTPUT,
#if 0
    const uint rpi = 10000,
#else
    const uint rpi = 100000,
#endif
    const DataByteSize data_size = DataByteSize::FULL_POSITION,
    const LogLevel log_level = LogLevel::INFO
  );
  ~PositionController();

  std::shared_ptr<SlaveUnit> register_slave_unit(
    const std::string& device_ip,
    const uint port = 0xAF12
  );

  bool execute();
  void wait();

private:
  void cleanup();

  ConnectionParameters connection_parameters_;
  ConnectionManager connection_manager_;
  std::shared_ptr<std::thread> controller_th_;

  std::map< std::shared_ptr<SessionInfo>, std::shared_ptr<IOConnection> > slaves_;

  int active_;
  DataByteSize data_size_;

};

}

#endif /* _IAI_POSITION_CONTROLLER_MASTER_UNIT_H_ */

