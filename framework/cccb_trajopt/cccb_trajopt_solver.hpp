#pragma once

#include "framework/user_command.hpp"
#include "rossy_utils/thirdparty/clp/clpwrapper.hpp"

// functions on cccb splines
class ObstacleManager;
class CCCBTrajManager;
class LPSolver;
class Clock;


class CCCBTrajOptSolver{
    public:
        CCCBTrajOptSolver();
        ~CCCBTrajOptSolver(){ delete lpsolver_; }
        bool solve(PLANNING_COMMAND* planning_cmd, 
                ObstacleManager* obstacles, 
                CCCBTrajManager* cccb_traj_);

    public:
        LPSolver* lpsolver_;
        Clock* clock_;

};