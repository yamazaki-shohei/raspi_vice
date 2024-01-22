#include <thread>

#include "vice/iai_position_controller/slave_unit.h"

using namespace eipScanner::cip;
using eipScanner::utils::Logger;

using namespace::iai_position_controller;

SlaveUnit::SlaveUnit(
        std::weak_ptr<IOConnection> io,
        uint8_t data_size,
        const LogLevel log_level
        )
{
  Logger::setLogLevel(log_level);

  if (data_size !=32) {
    Logger(LogLevel::ERROR) << "slave # not expected data size";
    return;
  }
  io_ = io;
  data_size_ = data_size;
  if (! setup()) { return; }
}

SlaveUnit::~SlaveUnit()
{
}

bool SlaveUnit::setup()
{
  req_.resize(data_size_);
  res_.resize(data_size_);
  for (size_t i = 0; i < data_size_; i++) {
    req_[i] = 0x00;
    res_[i] = 0x00;
  }
  Logger(LogLevel::DEBUG) << "slave # setup : " << req_.size();

  if (auto ptr = io_.lock()) {
    ptr->setDataToSend(req_);
    Logger(LogLevel::DEBUG) << "slave # setup : set data to send";
    ptr->setSendDataListener(
      [&](auto data) {
        std::ostringstream ss;
        ss << "data=";
        for (auto &byte : data) {
          ss << "[" << std::hex << (int) byte << "]";
        }
        Logger(LogLevel::DEBUG) << "slave # send: " << ss.str();
      });
    ptr->setReceiveDataListener(
      [&](auto realTimeHeader, auto sequence, auto data) {
        std::ostringstream ss;
        ss << "secNum=" << sequence << " data=";
        size_t index = 0;
        for (auto &byte : data) {
          ss << "[" << std::hex << (int) byte << "]";
          res_[index] = static_cast<uint8_t>(byte);
          index++;
        }
        Logger(LogLevel::DEBUG) << "slave # received: " << ss.str();
      });
    ptr->setCloseListener(
      [&]() {
        Logger(LogLevel::INFO) << "slave # closed";
      });
  } else {
    Logger(LogLevel::DEBUG) << "slave # setup : =====";
  }

  Logger(LogLevel::INFO) << "slave # setup exit";
  return true;
}

void SlaveUnit::request_update()
{
  if (auto ptr = io_.lock()) {
    ptr->setDataToSend(std::vector<uint8_t>(std::begin(req_), std::end(req_)));
  }
}

void SlaveUnit::request_control(uint8_t order[], uint8_t mode)
{
  if (auto ptr = io_.lock()) {
    switch (mode) {
    case 0: // override
      req_[data_size_-4] = order[0];
      req_[data_size_-3] = order[1];
      req_[data_size_-2] = order[2];
      req_[data_size_-1] = order[3];
      break;
    case 1: // on
      req_[data_size_-4] |= order[0];
      req_[data_size_-3] |= order[1];
      req_[data_size_-2] |= order[2];
      req_[data_size_-1] |= order[3];
      break;
    case 2: // off
      req_[data_size_-4] &= order[0];
      req_[data_size_-3] &= order[1];
      req_[data_size_-2] &= order[2];
      req_[data_size_-1] &= order[3];
    default:
      break;
    }
    ptr->setDataToSend(std::vector<uint8_t>(std::begin(req_), std::end(req_)));
  }
}

bool SlaveUnit::done_control(uint8_t mask[], uint8_t expect[])
{
  bool rt = false;
  for (int i = 0; i < 200; i++) {
    if (res_[data_size_-2] & 0x80) {
      Logger(LogLevel::WARNING) << "slave # detected EMERGENCY";
      break;
    }
    if (res_[data_size_-1] & 0x08) {
      uint16_t code = res_[data_size_-18];
      code <<= 4;
      code |= res_[data_size_-17];
      std::ostringstream ss;
      ss << "[" << std::hex << (uint16_t) code << "]";
      Logger(LogLevel::WARNING) << "slave # detected ALARM" << ss.str();
      break;
    }
    if ((res_[data_size_-4] & mask[0]) == expect[0] &&
        (res_[data_size_-3] & mask[1]) == expect[1] &&
        (res_[data_size_-2] & mask[2]) == expect[2] &&
        (res_[data_size_-1] & mask[3]) == expect[3]) {
      rt = true;
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  if (! rt) {
    Logger(LogLevel::DEBUG) << "slave # failed control";
  }
  return rt;
}

bool SlaveUnit::dstr_on()
{
  uint8_t order[] = { 0x00, 0x00, 0x00, 0x01 };
  request_control(order, 1);
  uint8_t mask[]  = { 0x00, 0x00, 0x00, 0x01 };
  return done_control(mask, mask);
}

bool SlaveUnit::dstr_off()
{
  uint8_t order[]  = { 0xFF, 0xFF, 0xFF, 0xFE };
  request_control(order, 2);
  uint8_t mask[]   = { 0x00, 0x00, 0x00, 0x01 };
  uint8_t expect[] = { 0x00, 0x00, 0x00, 0x00 };
  return done_control(mask, expect);
}

bool SlaveUnit::home()
{
  uint8_t on[] = { 0x00, 0x00, 0x00, 0x02 };
  request_control(on, 1);
  uint8_t mask[] = { 0x00, 0x00, 0x00, 0x02 };
  if (! done_control(mask, mask)) { return false; }
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  uint8_t off[] = { 0xFF, 0xFF, 0xFF, 0xFD };
  request_control(off, 2);
  return true;
}

bool SlaveUnit::stop_on()
{
  uint8_t order[] = { 0x00, 0x00, 0x00, 0x04 };
  request_control(order, 1);
  return true;
}

bool SlaveUnit::stop_off()
{
  uint8_t order[] = { 0xFF, 0xFF, 0xFF, 0xFB };
  request_control(order, 2);
  return true;
}

bool SlaveUnit::reset_on()
{
  uint8_t order[] = { 0x00, 0x00, 0x00, 0x08 };
  request_control(order, 0);
  return true;
}

bool SlaveUnit::reset_off()
{
  uint8_t order[] = { 0xFF, 0xFF, 0xFF, 0xF7 };
  request_control(order, 2);
  return true;
}

bool SlaveUnit::servo_on()
{
  uint8_t order[] = { 0x00, 0x00, 0x00, 0x10 };
  request_control(order, 1);
  uint8_t mask[]  = { 0x00, 0x00, 0x00, 0x10 };
  return done_control(mask, mask);
}

bool SlaveUnit::servo_off()
{
  uint8_t order[] = { 0xFF, 0xFF, 0xFF, 0xEF };
  request_control(order, 2);
  uint8_t mask[]   = { 0x00, 0x00, 0x00, 0x10 };
  uint8_t expect[] = { 0x00, 0x00, 0x00, 0x00 };
  return done_control(mask, expect);
}

bool SlaveUnit::jog()
{
  uint8_t order[] = { 0xFF, 0xFF, 0xFF, 0xDF };
  request_control(order, 2);
  return true;
}

bool SlaveUnit::inching()
{
  uint8_t order[] = { 0x00, 0x00, 0x00, 0x20 };
  request_control(order, 1);
  return true;
}

bool SlaveUnit::jog_velocity()
{
  uint8_t order[] = { 0xFF, 0xFF, 0xFF, 0xBF };
  request_control(order, 2);
  return true;
}

bool SlaveUnit::inching_velocity()
{
  uint8_t order[] = { 0x00, 0x00, 0x00, 0x40 };
  request_control(order, 1);
  return true;
}

bool SlaveUnit::jog_minus_on()
{
  uint8_t order[] = { 0x00, 0x00, 0x00, 0x80 };
  request_control(order, 1);
  return true;
}

bool SlaveUnit::jog_minus_off()
{
  uint8_t order[] = { 0xFF, 0xFF, 0xFF, 0x7F };
  request_control(order, 2);
  return true;
}

bool SlaveUnit::jog_plus_on()
{
  uint8_t order[] = { 0x00, 0x00, 0x01, 0x00 };
  request_control(order, 1);
  return true;
}

bool SlaveUnit::jog_plus_off()
{
  uint8_t order[] = { 0xFF, 0xFF, 0xFE, 0xFF };
  request_control(order, 2);
  return true;
}

bool SlaveUnit::brake_release()
{
  uint8_t order[] = { 0x00, 0x00, 0x80, 0x00 };
  request_control(order, 0);
  return true;
}

bool SlaveUnit::position_on()
{
  uint8_t order[] = { 0xFF, 0xFD, 0xFF, 0xFF };
  request_control(order, 2);
  return true;
}

bool SlaveUnit::push_on()
{
  uint8_t order[] = { 0x00, 0x02, 0x00, 0x00 };
  request_control(order, 1);
  return true;
}

bool SlaveUnit::dir_sub()
{
  uint8_t order[] = { 0xFF, 0xFB, 0xFF, 0xFF };
  request_control(order, 2);
  return true;
}

bool SlaveUnit::dir_add()
{
  uint8_t order[] = { 0x00, 0x04, 0x00, 0x00 };
  request_control(order, 1);
  return true;
}

bool SlaveUnit::inc_off()
{
  uint8_t order[] = { 0xFF, 0xF7, 0xFF, 0xFF };
  request_control(order, 2);
  return true;
}

bool SlaveUnit::inc_on()
{
  uint8_t order[] = { 0x00, 0x08, 0x00, 0x00 };
  request_control(order, 1);
  return true;
}

bool SlaveUnit::position(int32_t val)
{
  if (val < -999999 || val > 999999) {
    Logger(LogLevel::ERROR) << "slave # position not expected value";
    return false;
  }
  for (int i = 0; i < 4; i++) { req_[4*0+i] = ((uint8_t *)&val)[3-i]; }
#if 1
  std::ostringstream ss;
  for (int i = 0; i < 4; i++) {
    ss << "[" << std::hex << (int) req_[i] << "]";
  }
  Logger(LogLevel::INFO) << "slave # position: " << val << " " << ss.str();
#endif
  request_update();
  return true;
}

bool SlaveUnit::width(int32_t val)
{
  if (val < 0 || val > 999999) {
    Logger(LogLevel::ERROR) << "slave # width not expected value";
    return false;
  }
  for (int i = 0; i < 4; i++) { req_[4*1+i] = ((uint8_t *)&val)[3-i]; }
  request_update();
  return true;
}

bool SlaveUnit::velocity(int32_t val)
{
  if (val < 0 || val > 999999) {
    Logger(LogLevel::ERROR) << "slave # velocity not expected value";
    return false;
  }
  for (int i = 0; i < 4; i++) { req_[4*2+i] = ((uint8_t *)&val)[3-i]; }
  request_update();
  return true;
}

bool SlaveUnit::zone_plus(int32_t val)
{
  if (val < -999999 || val > 999999) {
    Logger(LogLevel::ERROR) << "slave # zone_plus not expected value";
    return false;
  }
  for (int i = 0; i < 4; i++) { req_[4*3+i] = ((uint8_t *)&val)[3-i]; }
  request_update();
  return true;
}

bool SlaveUnit::zone_minus(int32_t val)
{
  if (val < -999999 || val > 999999) {
    Logger(LogLevel::ERROR) << "slave # zone_minus not expected value";
    return false;
  }
  for (int i = 0; i < 4; i++) { req_[4*4+i] = ((uint8_t *)&val)[3-i]; }
  request_update();
  return true;
}

bool SlaveUnit::acceleration(int16_t plus, int16_t minus)
{
  if (plus < 0 || plus > 300 || minus < 0 || minus > 300) {
    Logger(LogLevel::ERROR) << "slave # acceleration not expected value";
    return false;
  }
  for (int i = 0; i < 2; i++) { req_[4*5+i] = ((uint8_t *)&plus)[1-i]; }
  for (int i = 0; i < 2; i++) { req_[4*5+2+i] = ((uint8_t *)&minus)[1-i]; }
  request_update();
  return true;
}

bool SlaveUnit::push_currents(int16_t val)
{
  if (val < 0 || val > 255) {
    Logger(LogLevel::ERROR) << "slave # push currents not expected value";
    return false;
  }
  val *= 100;
  val /= 255;
  for (int i = 0; i < 2; i++) { req_[4*6+i] = ((uint8_t *)&val)[1-i]; }
  request_update();
  return true;
}

bool SlaveUnit::load_currents(int16_t val)
{
  if (val < 0 || val > 255) {
    Logger(LogLevel::ERROR) << "slave # load currents not expected value";
    return false;
  }
  val *= 100;
  val /= 255;
  for (int i = 0; i < 2; i++) { req_[4*6+2+i] = ((uint8_t *)&val)[1-i]; }
  request_update();
  return true;
}

bool SlaveUnit::move_to_push(
  int32_t p, int32_t w, int32_t v, int16_t ap, int16_t am, int16_t pc
  )
{
  position(p);
  width(w);
  velocity(v);
  acceleration(ap, am);
  push_currents(pc);
  push_on();
  dir_add();
  dstr_on();
  uint8_t mask[] = { 0x00, 0x00, 0x02, 0x40 };
  return done_control(mask, mask);
}

