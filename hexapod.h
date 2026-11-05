/* Portable 3-DOF kinematics and phase generator. No hardware or scheduler. */
#ifndef HEXAPOD_H
#define HEXAPOD_H
#include <stdint.h>
#include <stdbool.h>
#define HEX_LEGS 6U
#define HEX_JOINTS 18U
typedef enum { HEX_TRIPOD, HEX_RIPPLE, HEX_WAVE } hex_gait_t;
typedef struct { float x, y, z; } hex_point_t;
typedef struct {
    float coxa_mm, femur_mm, tibia_mm;
    hex_point_t neutral;
    /* Body axes X forward, Y left, Z down. Local X points radially out. */
    float mount_deg[HEX_LEGS];
    float hip_x_mm[HEX_LEGS], hip_y_mm[HEX_LEGS];
    uint8_t id[HEX_JOINTS];
    float neutral_deg[HEX_JOINTS];
    int8_t direction[HEX_JOINTS];
    float min_deg[HEX_JOINTS], max_deg[HEX_JOINTS];
    bool calibrated;
} hex_config_t;
bool hex_ik(const hex_config_t *cfg, hex_point_t foot, float angle[3]);
bool hex_config_valid(const hex_config_t *cfg);
bool hex_pose(const hex_config_t *cfg, const hex_point_t feet[HEX_LEGS], float degrees[HEX_JOINTS]);
/* phase [0,1); stride in mm/cycle, yaw in degrees/cycle. */
bool hex_feet(const hex_config_t *cfg, hex_gait_t gait, float phase,
              float forward_mm, float left_mm, float yaw_deg, float lift_mm,
              hex_point_t feet[HEX_LEGS]);
#endif
