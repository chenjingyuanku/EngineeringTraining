#include "key.h"
#include "multi_button.h"
#include "stm32f1xx_hal.h"
#include "main.h"
#include "ui.h"
#include "control.h"
#include "at24c02.h"

Button btn_up;
Button btn_down;
Button btn_page_up;
Button btn_page_down;
Button btn_add;
Button btn_sub;
Button btn_enter;
Button btn_return;

uint8_t read_key_up()
{
    return HAL_GPIO_ReadPin(key_up_GPIO_Port, key_up_Pin);
}
uint8_t read_key_down()
{
    return HAL_GPIO_ReadPin(key_down_GPIO_Port, key_down_Pin);
}
uint8_t read_key_add()
{
    return HAL_GPIO_ReadPin(key_add_GPIO_Port, key_add_Pin);
}
uint8_t read_key_sub()
{
    return HAL_GPIO_ReadPin(key_sub_GPIO_Port, key_sub_Pin);
}
uint8_t read_key_run()
{
    return HAL_GPIO_ReadPin(key_run_GPIO_Port, key_run_Pin);
}
uint8_t read_key_save()
{
    return HAL_GPIO_ReadPin(key_save_GPIO_Port, key_save_Pin);
}
uint8_t read_key_page_up()
{
    return HAL_GPIO_ReadPin(key_page_up_GPIO_Port, key_page_up_Pin);
}
uint8_t read_key_page_down()
{
    return HAL_GPIO_ReadPin(key_page_down_GPIO_Port, key_page_down_Pin);
}


void key_up_click(void* btn)
{
    control.item_index ++;
    switch(control.page_index)
    {
        case param_page: 
            break;
        case place_setting_page:
            if(control.task == 0)
            {
                if(control.item_index >= 3)
                    control.item_index = 3;
            }
            if(control.item_index >= 6)
                    control.item_index = 6;
            break;
        case pid_page:
            if(control.item_index >= 5)
                    control.item_index = 5;
            break;
        case baffle_page:
            if(control.item_index >= 5)
                    control.item_index = 5;
            break;
        case uphill_page:
            if(control.item_index >= 1)
                    control.item_index = 1;
            break;
        case get_goods_time_page:
        case lift_goods_time_page:
        case place_goods_time_page:
        case get_and_lift_goods_time_page:
            if(control.item_index >= 5)
                    control.item_index = 5;
            break;
            
    }
    beep_flag = 1;
}
void key_down_click(void* btn)
{
    if(control.item_index > 0)
        control.item_index --;
    beep_flag = 1;
}
void key_add_click(void* btn)
{
    switch(control.page_index)
    {
        case param_page:
            break;
        case place_setting_page:
            if(control.item_index > 0)
            {
                if(control.task == 0)
                {
                    place_position_task0[control.item_index - 1] ++;
                    if(place_position_task0[control.item_index - 1] > 3)
                        place_position_task0[control.item_index - 1] = 3;
                }
                else
                {
                    place_position[control.item_index - 1] ++;
                    if(place_position[control.item_index - 1] > 6)
                        place_position[control.item_index - 1] = 6;
                }
            }
            else
            {
                control.task ++;
                if(control.task > 2)
                    control.task = 2;
            }
            break;
        case pid_page:
            switch(control.item_index)
            {
                case 0:pid.kp_l += 10;
                    break;
                case 1:pid.kd_l += 10;
                    break;
                case 2:pid.kp_r += 10;
                    break;
                case 3:pid.kd_r += 10;
                    break;
                case 4:control.speed += 10;
                    break;
                case 5:control.right_wheel_speed_offset += 2;
                    break;
            }
            break;
        case baffle_page:
            switch(control.item_index)
            {
                case 0:first_station_stop_speed += 10;
                    break;
                case 1:baffle_speed += 10;
                    break;
                case 2:turn_right_time += 10;
                    break;
                case 3:turn_right_spd += 10;
                    break;
                case 4:turn_left_time += 10;
                    break;
                case 5:turn_left_spd += 10;
                    break;
                
            }
            break;
        case uphill_page:
            switch(control.item_index)
            {
                case 0:uphill_speed += 10;
                    break;
                case 1:second_station_speed += 10;
                    break;
            }
            
            break;
        case get_goods_time_page:
            get_goods_time[control.item_index] += 50;
            break;
        case lift_goods_time_page:
            lift_goods_time[control.item_index] += 50;
            break;
        case place_goods_time_page:
            place_goods_time[control.item_index] += 50;
            break;
        case get_and_lift_goods_time_page:
            get_and_lift_goods_time[control.item_index] += 50;
            break;
    }
    beep_flag = 1;
}

void key_sub_click(void* btn)
{
    switch(control.page_index)
    {
        case param_page:
            break;
        case place_setting_page:
            if(control.item_index > 0)
            {
                
                if(control.task == 0)
                {
                    if(place_position_task0[control.item_index - 1] > 1)
                        place_position_task0[control.item_index - 1] --;
                }
                else
                {
                    if(place_position[control.item_index - 1] > 1)
                        place_position[control.item_index - 1] --;
                }
            }
            else
            {
                if(control.task > 0)
                    control.task --;
            }
            break;
        case pid_page:
            switch(control.item_index)
            {
                case 0:pid.kp_l -= 10;
                    break;
                case 1:pid.kd_l -= 10;
                    break;
                case 2:pid.kp_r -= 10;
                    break;
                case 3:pid.kd_r -= 10;
                    break;
                case 4:control.speed -= 10;
                    break;
                case 5:control.right_wheel_speed_offset -= 2;
                    break;
            }
            break;
        case baffle_page:
            switch(control.item_index)
            {
                case 0:
                    if(first_station_stop_speed >= 10)
                        first_station_stop_speed -= 10;
                    break;
                case 1:
                    if(baffle_speed >= 10)
                        baffle_speed -= 10;
                    break;
                case 2:
                    if(turn_right_time >= 10)
                        turn_right_time -= 10;
                    break;
                case 3:
                    if(turn_right_spd >= 10)
                        turn_right_spd -= 10;
                    break;
                case 4:
                    if(turn_left_time >= 10)
                        turn_left_time -= 10;
                    break;
                case 5:
                    if(turn_left_spd >= 10)
                        turn_left_spd -= 10;
                    break;
            }
            break;
        case uphill_page:
            switch(control.item_index)
            {
                case 0:
                    if(uphill_speed >= 10)
                        uphill_speed -= 10;
                    break;
                case 1:
                if(second_station_speed >= 10)
                    second_station_speed -= 10;
                    break;
            }
            break;
        case get_goods_time_page:
            if(get_goods_time[control.item_index] >= 1050)
                get_goods_time[control.item_index] -= 50;
            break;
        case lift_goods_time_page:
            if(lift_goods_time[control.item_index] >= 1050)
                lift_goods_time[control.item_index] -= 50;
            break;
        case place_goods_time_page:
            if(place_goods_time[control.item_index] >= 1050)
                place_goods_time[control.item_index] -= 50;
            break;
        case get_and_lift_goods_time_page:
            if(get_and_lift_goods_time[control.item_index] >= 1050)
                get_and_lift_goods_time[control.item_index] -= 50;
            break;
    }
    beep_flag = 1;
}
void key_enter_click(void* btn)
{
    control.run_flag = 1 - control.run_flag;
//    control.run_mode = 9;
//    current_step = baffle;
    beep_flag = 1;
}
void key_save_click(void* btn)
{
    save_flag = 1;
    beep_flag = 1;
}
void key_page_up_click(void* btn)
{
    if(control.page_index + 1 < MAX_PAGE)
        control.page_index ++;
    control.item_index = 0;
    beep_flag = 1;
}
void key_page_down_click(void* btn)
{
    
    if(control.page_index > 0)
        control.page_index --;
    control.item_index = 0;
    beep_flag = 1;
}
void key_init(void)
{
    //按钮驱动注册
    button_init(&btn_up, read_key_up, 0);
    button_init(&btn_down, read_key_down, 0);
    button_init(&btn_add, read_key_add, 0);
    button_init(&btn_sub, read_key_sub, 0);
    button_init(&btn_enter, read_key_run, 0);
    button_init(&btn_return, read_key_save, 0);
    button_init(&btn_page_up, read_key_page_up, 0);
    button_init(&btn_page_down, read_key_page_down, 0);
    //按钮单击事件注册
    button_attach(&btn_up, SINGLE_CLICK, key_up_click);
    button_attach(&btn_down, SINGLE_CLICK, key_down_click);
    button_attach(&btn_up, LONG_PRESS_HOLD, key_up_click);
    button_attach(&btn_down, LONG_PRESS_HOLD, key_down_click);
    button_attach(&btn_add, SINGLE_CLICK, key_add_click);
    button_attach(&btn_sub, SINGLE_CLICK, key_sub_click);
    button_attach(&btn_add, LONG_PRESS_HOLD, key_add_click);
    button_attach(&btn_sub, LONG_PRESS_HOLD, key_sub_click);
    button_attach(&btn_enter, SINGLE_CLICK, key_enter_click);
    button_attach(&btn_return, SINGLE_CLICK, key_save_click);
    button_attach(&btn_page_up, SINGLE_CLICK, key_page_up_click);
    button_attach(&btn_page_down, SINGLE_CLICK, key_page_down_click);
    button_attach(&btn_page_up, LONG_PRESS_HOLD, key_page_up_click);
    button_attach(&btn_page_down, LONG_PRESS_HOLD, key_page_down_click);
    //启动按钮事件
    button_start(&btn_up);
    button_start(&btn_down);
    button_start(&btn_page_up);
    button_start(&btn_page_down);
    button_start(&btn_add);
    button_start(&btn_sub);
    button_start(&btn_enter);
    button_start(&btn_return);
    /*记得定时调用button_ticks()函数*/
    /*记得定时调用button_ticks()函数*/
}
