#ifndef RobotLocalization_FilterCommon_h
#define RobotLocalization_FilterCommon_h

namespace RobotLocalization
{
//Enumeration that defines the state vector//3-POSITION//3-ANGLE//3-VELOCITY//3-ANGULAR_VELOCITY
  enum StateMembers
  {
    StateMemberX = 0,
    StateMemberY,
    StateMemberZ,
    StateMemberRoll,
    StateMemberPitch,
    StateMemberYaw,
    StateMemberVx,
    StateMemberVy,
    StateMemberVz,
    StateMemberVroll,
    StateMemberVpitch,
    StateMemberVyaw
  };

//Global constants that define our state vector size and offsets to groups of values within that state.
  const int STATE_SIZE = 12;
  const int POSITION_OFFSET = StateMemberX;
  const int ORIENTATION_OFFSET = StateMemberRoll;		
  const int POSITION_V_OFFSET = StateMemberVx;		//VELOCITY OFFSET
  const int ORIENTATION_V_OFFSET = StateMemberVroll;	//ANGULAR VELOCITY OFFSET

//Pose and twist messages each contain six variables
  const int POSE_SIZE = 6;
  const int TWIST_SIZE = 6;
  const int POSITION_SIZE = 3;
  const int ORIENTATION_SIZE = 3;
}

#endif
