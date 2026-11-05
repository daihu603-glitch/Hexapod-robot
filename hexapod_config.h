/* EXAMPLE ONLY: measure your robot before setting calibrated=true.
 * Neutral angles mean the servo commands at the specified neutral foot position,
 * NOT the mathematical zero of the joint. ID order MUST be checked on hardware. */
#ifndef HEXAPOD_CONFIG_H
#define HEXAPOD_CONFIG_H
#include "hexapod.h"
#define HEX_SIX(v) v,v,v,v,v,v
#define HEX_EIGHTEEN(v) HEX_SIX(v),HEX_SIX(v),HEX_SIX(v)
static const hex_config_t hex_robot_config = {
    30, 60, 100, {100, 0, 80},
    {-45,-90,-135,45,90,135},
    {60,0,-60,60,0,-60}, {-45,-55,-45,45,55,45},
    {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17},
    {HEX_EIGHTEEN(90)}, {HEX_EIGHTEEN(1)},
    {HEX_EIGHTEEN(45)}, {HEX_EIGHTEEN(135)},
    false
};
#endif
