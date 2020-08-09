#include <rosilo_robot_driver/rosilo_robot_driver.h>

namespace rosilo
{

RobotDriver::RobotDriver(std::atomic_bool *break_loops):
    break_loops_(break_loops)
{

}

}
