#ifndef APP_MOTION_H
#define APP_MOTION_H
#include <stddef.h>
#include <stdint.h>
/* All functions owned by servo task; no other task may call these. */
void app_motion_command(const char *command, char *reply, size_t capacity, uint32_t now_ms);
void app_motion_process(uint32_t now_ms);
void app_motion_cancel(void);
#endif
