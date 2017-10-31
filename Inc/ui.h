#ifndef _ui_h
#define _ui_h 
#include "stm32f1xx_hal.h"

void ui_display(void);

enum menu_page_num
{
    param_page,
    place_setting_page,
    pid_page,
    baffle_page,
    uphill_page,
    get_goods_time_page,
    lift_goods_time_page,
    place_goods_time_page,
    get_and_lift_goods_time_page,
    fixer_duty_setting_page,
    action_speed_mode_setting_page,
    MAX_PAGE
};

#endif
