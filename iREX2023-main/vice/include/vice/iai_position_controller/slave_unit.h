#ifndef _IAI_POSITION_CONTROLLER_SLAVE_UNIT_H_
#define _IAI_POSITION_CONTROLLER_SLAVE_UNIT_H_

#include <string>
#include <vector>

#include "EIPScanner/IOConnection.h"
#include "EIPScanner/utils/Logger.h"

using eipScanner::IOConnection;
using eipScanner::utils::LogLevel;

namespace iai_position_controller
{

class SlaveUnit
{

public:

  explicit SlaveUnit(
    std::weak_ptr<IOConnection> io,
    uint8_t data_size,
    const LogLevel log_level = LogLevel::INFO
  );

  ~SlaveUnit();

  void request_update();
  void request_control(uint8_t order[], uint8_t mode);
  bool done_control(uint8_t mask[], uint8_t expect[]);

  bool dstr_on();
  bool dstr_off();
  bool home();
  bool stop_on();
  bool stop_off();
  bool reset_on();
  bool reset_off();
  bool servo_on();
  bool servo_off();
  bool jog();
  bool inching();
  bool jog_velocity();
  bool inching_velocity();
  bool jog_minus_on();
  bool jog_minus_off();
  bool jog_plus_on();
  bool jog_plus_off();
  bool brake_release();
  bool position_on();
  bool push_on();
  bool dir_sub();
  bool dir_add();
  bool inc_off();
  bool inc_on();

  bool position(int32_t val);
  bool width(int32_t val);
  bool velocity(int32_t val);
  bool zone_plus(int32_t val);
  bool zone_minus(int32_t val);

  bool acceleration(int16_t plus, int16_t minus);
  bool push_currents(int16_t val);
  bool load_currents(int16_t val);

  bool move_to_push(
        int32_t p, int32_t w, int32_t v, int16_t ap, int16_t am, int16_t pc
        );

private:

  bool setup();

  std::weak_ptr<IOConnection> io_;
  uint8_t data_size_;
  std::vector<uint8_t> req_;
  std::vector<uint8_t> res_;

};

}

#endif /* _IAI_POSITION_CONTROLLER_SLAVE_UNIT_H_ */

