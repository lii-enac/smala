use core
use exec_env
use base

_native_code_
%{
extern "C" {
#include "commander.h"
#include "debug.h"
}

static void setHoverSetpoint(setpoint_t *setpoint, float vx, float vy, float z, float yawrate)
{
  setpoint->mode.z = modeAbs;
  setpoint->position.z = z;

  setpoint->mode.yaw = modeVelocity;
  setpoint->attitudeRate.yaw = yawrate;

  setpoint->mode.x = modeVelocity;
  setpoint->mode.y = modeVelocity;
  setpoint->velocity.x = vx;
  setpoint->velocity.y = vy;

  setpoint->velocity_body = true;
}

setpoint_t setpoint;

void
hover(CoreProcess* p)
{
    float velFront=0, velSide=0, height=0.5;
    setHoverSetpoint(&setpoint, velFront, velSide, height, 0);
    commanderSetSetpoint(&setpoint, 3);
    //DEBUG_PRINT("hover\n");
}

void
stop_hover(CoreProcess* p)
{
    float velFront=0, velSide=0, height=0;
    setHoverSetpoint(&setpoint, velFront, velSide, height, 0);
    commanderSetSetpoint(&setpoint, 3);
    //DEBUG_PRINT("hover\n");
}

%}

_define_
crazyflie_drone_app()
{
    NativeAction hover_na (hover, this, 1)
    NativeAction stop_hover_na (stop_hover, this, 1)
    FSM truc {
        State idle {
            Timer t(3000)
            |->stop_hover_na
        }
        State hover {
            Timer t(3000)
            |->hover_na
        }
        idle->hover (idle.t.end)
        hover->idle (hover.t.end)
    }
    TextPrinter tp
    truc.state =:> tp.input
}