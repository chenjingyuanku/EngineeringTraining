#include "ui.h"
#include "oled.h"
#include "control.h"
#include "adc.h"
#include "at24c02.h"
#include "stm32f1xx_hal.h"

void saved_tip(void)
{
    static uint64_t cnt = 0;
    if(save_show_flag)
    {
        cnt = system_runtime_ms;
        save_show_flag = 0;
    }
    if(system_runtime_ms < (cnt + 1000))
        oled_printf(15,0,"saved");
}

char place_setting_list[6][5]={"DN L","DN M","DN R","UP L","UP M","UP R"};
void ui_display(void)
{
    
    oled_printf(0,0,"Page %d",control.page_index);
    
    //启动状态显示
    if(control.run_flag)
        oled_printf(9,0,">>>");
    else
        oled_printf(9,0,"---");
    
    
    
    
    oled_printf(0,1,"%d",control.run_mode);
    //车头两个灰度传感器
    if(is_IR_sensor_valid)
    {
        oled_printf(9,1,"---");
    }
        
    if(HAL_GPIO_ReadPin(head2_GPIO_Port,head2_Pin) == GPIO_PIN_SET)
    {
        oled_printf(4,1,"==<");
    }
    if(HAL_GPIO_ReadPin(head1_GPIO_Port,head1_Pin) == GPIO_PIN_SET)
    {
        oled_printf(14,1,">==");
    }
    if(HAL_GPIO_ReadPin(right_sensor_GPIO_Port,right_sensor_Pin) == GPIO_PIN_SET)
    {
        oled_printf(18,1,"<");
    }
    
    //显示保存成功提示
    saved_tip();
    
    switch(control.page_index)
    {
        
        case param_page:
            oled_printf(0 ,2,"ad   nor  min   max");

            oled_printf(0 ,3,"%d",adc_values[0]);
            oled_printf(0 ,4,"%d",adc_values[1]);
            oled_printf(0 ,5,"%d",adc_values[2]);
            
            oled_printf(5 ,3,"%.0f",ad1_nor);
            oled_printf(5 ,4,"%.0f",ad2_nor);
            oled_printf(5 ,5,"%.0f",ad3_nor);
        
            oled_printf(9 ,3," %.0f",ad1_min);
            oled_printf(9 ,4," %.0f",ad2_min);
            oled_printf(9 ,5," %.0f",ad3_min);
        
            oled_printf(15 ,3," %.0f",ad1_max);
            oled_printf(15 ,4," %.0f",ad2_max);
            oled_printf(15 ,5," %.0f",ad3_max);
        
            oled_printf(0 ,6,"ERR    %.2f",pid.err);
            oled_printf(0 ,7,"PIDOUT %.2f",pid.pidout);
        
            oled_printf(16 ,6,"%d",left_speed);
            oled_printf(16 ,7,"%d",right_speed);
            break;
        case place_setting_page:
            oled_printf(5,2,"Bl Dn > %s",control.task == 0?place_setting_list[place_position_task0[0]-1]:place_setting_list[place_position[0]-1]);
            oled_printf(5,3,"Gr Dn > %s",control.task == 0?place_setting_list[place_position_task0[1]-1]:place_setting_list[place_position[1]-1]);
            oled_printf(5,4,"Rd Dn > %s",control.task == 0?place_setting_list[place_position_task0[2]-1]:place_setting_list[place_position[2]-1]);
            oled_printf(5,5,control.task == 0?"------------":"Bl Up > %s",place_setting_list[place_position[3]-1]);
            oled_printf(5,6,control.task == 0?"------------":"Gr Up > %s",place_setting_list[place_position[4]-1]);
            oled_printf(5,7,control.task == 0?"------------":"Rd Up > %s",place_setting_list[place_position[5]-1]);
            oled_printf(0,2,"Task");
            oled_printf(0,4," %d",control.task);
            if(control.item_index > 0)
                oled_printf(18,control.item_index + 1,"<");
            else
                oled_printf(0,6,"^^^");
            
            break;
        case pid_page:
            oled_printf(0,2,"kp_l   =  %.0f",pid.kp_l);
            oled_printf(0,3,"kd_l   =  %.0f",pid.kd_l);
            oled_printf(0,4,"kp_r   =  %.0f",pid.kp_r);
            oled_printf(0,5,"kd_r   =  %.0f",pid.kd_r);
            oled_printf(0,6,"speed  =  %d",control.speed);
            oled_printf(0,7,"offset =  %d",control.right_wheel_speed_offset);
            oled_printf(16,control.item_index + 2,"<");
            break;
        case baffle_page:
            
            oled_printf(0,2,"no1_stop_speed  %d",first_station_stop_speed);
            oled_printf(0,3,"baffle_speed    %d",baffle_speed);
            oled_printf(0,4,"turn_right_time %d",turn_right_time);
            oled_printf(0,5,"turn_right_spd  %d",turn_right_spd);
            oled_printf(0,6,"turn_left_time  %d",turn_left_time);
            oled_printf(0,7,"turn_left_spd   %d",turn_left_spd);
            oled_printf(15,control.item_index + 2,">");
            break;
        case uphill_page:
            oled_printf(0,2,"   uphill_speed");
            oled_printf(0,3,"        %d",uphill_speed);
            oled_printf(0,5,"second_station_speed");
            oled_printf(0,6,"        %d",second_station_speed);
            oled_printf(0,control.item_index == 0 ? 4 : 7,"      ^^^^^^^");
            break;
        case get_goods_time_page:
            oled_printf(0,2,"get down L    %d",get_goods_time[0]);
            oled_printf(0,3,"get down M    %d",get_goods_time[1]);
            oled_printf(0,4,"get down R    %d",get_goods_time[2]);
            oled_printf(0,5,"get up   L    %d",get_goods_time[3]);
            oled_printf(0,6,"get up   M    %d",get_goods_time[4]);
            oled_printf(0,7,"get up   R    %d",get_goods_time[5]);
            oled_printf(13,control.item_index + 2,">");
            break;
        case lift_goods_time_page:
            oled_printf(0,2,"lift down L    %d",lift_goods_time[0]);
            oled_printf(0,3,"lift down M    %d",lift_goods_time[1]);
            oled_printf(0,4,"lift down R    %d",lift_goods_time[2]);
            oled_printf(0,5,"lift up   L    %d",lift_goods_time[3]);
            oled_printf(0,6,"lift up   M    %d",lift_goods_time[4]);
            oled_printf(0,7,"lift up   R    %d",lift_goods_time[5]);
            oled_printf(14,control.item_index + 2,">");
            break;
        case place_goods_time_page:
            oled_printf(0,2,"place down L    %d",place_goods_time[0]);
            oled_printf(0,3,"place down M    %d",place_goods_time[1]);
            oled_printf(0,4,"place down R    %d",place_goods_time[2]);
            oled_printf(0,5,"place up   L    %d",place_goods_time[3]);
            oled_printf(0,6,"place up   M    %d",place_goods_time[4]);
            oled_printf(0,7,"place up   R    %d",place_goods_time[5]);
            oled_printf(15,control.item_index + 2,">");
            break;
        case get_and_lift_goods_time_page:
            oled_printf(0,2,"g & l down L    %d",get_and_lift_goods_time[0]);
            oled_printf(0,3,"g & l down M    %d",get_and_lift_goods_time[1]);
            oled_printf(0,4,"g & l down R    %d",get_and_lift_goods_time[2]);
            oled_printf(0,5,"g & l up   L    %d",get_and_lift_goods_time[3]);
            oled_printf(0,6,"g & l up   M    %d",get_and_lift_goods_time[4]);
            oled_printf(0,7,"g & l up   R    %d",get_and_lift_goods_time[5]);
            oled_printf(15,control.item_index + 2,">");
            break;
        
    }
    OLED_Updata();
    OLED_ClearScreen();
}
