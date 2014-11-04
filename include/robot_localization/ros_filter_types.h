#ifndef RobotLocalization_RosFilterTypes_h
#define RobotLocalization_RosFilterTypes_h

#include <robot_localization/ros_filter.h>
#include <robot_localization/ekf.h>

namespace RobotLocalization
{
  typedef RosFilter<Ekf> RosEkf;
}

#endif
