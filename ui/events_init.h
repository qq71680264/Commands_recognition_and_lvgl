/*
 * Copyright 2023 NXP
 * SPDX-License-Identifier: MIT
 * The auto-generated can only be used on NXP devices
 */


#ifndef EVENTS_INIT_H_
#define EVENTS_INIT_H_
#ifdef __cplusplus
extern "C" {
#endif

#include "gui_guider.h"

void events_init(lv_ui *ui);
void video_play(lv_ui *ui);
void events_init_screen(lv_ui *ui);
extern void Light_FUNC(void);

extern int Light_Flag;

#ifdef __cplusplus
}
#endif
#endif /* EVENT_CB_H_ */