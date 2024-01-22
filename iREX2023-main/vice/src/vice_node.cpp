#include <vector>

#include "ros/ros.h"
#include "vice/Vice.h"
#include "vice/iai_position_controller/position_controller.h"

using namespace iai_position_controller;
using iai_position_controller::PositionController;
using iai_position_controller::SlaveUnit;

class ViceController
{
public:
  ViceController();
  ~ViceController();
  bool order(vice::Vice::Request &req, vice::Vice::Response &res);
  bool setup_position_controller();
  bool reset();

private:
  PositionController pcon;
  std::vector< std::shared_ptr<SlaveUnit> > slaves;

};

ViceController::ViceController()
{
  setup_position_controller();
}

ViceController::~ViceController()
{
  ;
}

bool ViceController::order(vice::Vice::Request &req, vice::Vice::Response &res)
{
  ROS_INFO("vice_control: position=(%d, %d)", req.position_x, req.position_y);

  if (req.position_x == 0 && req.position_y == 0) {
    res.status = reset();
    return true;
  }

  for (auto& slave : slaves) {
    slave->velocity(500);
    slave->acceleration(10, 10);
    slave->push_currents(50);
    slave->push_on();
    slave->inc_off();
  }

  if (req.position_x > 0) {
    slaves[0]->dir_add();
    slaves[0]->position(req.position_x);
    slaves[0]->width(req.position_x);
  } else {
    slaves[0]->dir_sub();
    slaves[0]->position(req.position_x * -1);
    slaves[0]->width(req.position_x * -1);
  }

  if (req.position_y > 0) {
    slaves[1]->dir_add();
    slaves[1]->position(req.position_y);
    slaves[1]->width(req.position_y);
  } else {
    slaves[1]->dir_sub();
    slaves[1]->position(req.position_y * -1);
    slaves[1]->width(req.position_y * -1);
  }

  for (auto& slave : slaves) {
    slave->dstr_on();
  }

  uint8_t mask[] = { 0x00, 0x00, 0x00, 0x40 };
  res.status = true;

  for (auto& slave : slaves) {
    if (! slave->done_control(mask, mask)) {
      res.status = false;
    }
    slave->dstr_off();
  }

  return true;
}

bool ViceController::setup_position_controller()
{
  std::vector<std::string> slave_units;
  if (! ros::param::get("~/slave_units", slave_units)) {
    ROS_ERROR("vice_control: not defined slave_units");
    return false;
  }

  for (auto& slave_unit : slave_units) {
    std::cout << "slave_unit : " << slave_unit << std::endl;
    std::shared_ptr<SlaveUnit> slave = pcon.register_slave_unit(slave_unit);
    slaves.push_back(slave);
  }
  pcon.execute();

  return reset();
}

bool ViceController::reset()
{
  ROS_INFO("vice_control: call reset");
  ros::Rate r(2000);

  for (auto& slave : slaves) {
    slave->reset_on();
  }
  r.sleep();

  for (auto& slave : slaves) {
    slave->reset_off();
    if (! slave->servo_on()) {
      ROS_ERROR("vice_control: can not turn on servo");
      return false;
    }
  }
  r.sleep();

  for (auto& slave : slaves) {
    if (! slave->home()) {
      ROS_ERROR("vice_control: can not move to home");
      return false;
    }
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "vice_control");
  ros::NodeHandle n("~");

  ViceController vice_controller;
  ros::ServiceServer service = n.advertiseService(
    "controller", &ViceController::order, &vice_controller
    );

  ros::spin();

  return 0;
}

