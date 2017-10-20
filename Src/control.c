/*
 * @CreateTime: Sep 16, 2017 6:23 PM
 * @Author: undefined
 * @Contact: undefined
 * @Last Modified By: undefined
 * @Last Modified Time: Sep 16, 2017 6:24 PM
 * @Description: Modify Here, Please 
 */
#include "control.h"
#include "tim.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "at24c02.h"
#include "oled.h"
#include "usart.h"

#define stop_motor motor(0,0,0)
#define set_run_mode(n) {stop_motor;turn_time = system_runtime_ms;control.run_mode=n;}
#define next_run_mode {stop_motor;turn_time = system_runtime_ms;control.run_mode++;}
#define is_time_out_ms(n) (turn_time + n < system_runtime_ms)
//                 速度方向，转动时间，下一个case
#define robot_turn(spd,tm,n) {motor(spd,-(spd),1);if((turn_time+tm)<system_runtime_ms){set_run_mode(n);}}
#define robot_turn_next(spd,tm) {motor(spd,-(spd),1);if((turn_time+tm)<system_runtime_ms){next_run_mode;}}
#define unfixed {__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,up_unfixed_duty);__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,down_unfixed_duty);}
#define fixed {__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_3,up_fixed_duty);__HAL_TIM_SET_COMPARE(&htim2,TIM_CHANNEL_4,down_fixed_duty);}




uint16_t uphill_speed = 650;
uint16_t baffle_speed = 400;
uint16_t turn_right_time = 330;
uint16_t turn_right_spd = 800;
uint16_t turn_left_time = 200;
uint16_t turn_left_spd = 600;
uint16_t second_station_speed = 200;
uint16_t first_station_stop_speed = 150;


control_type control;
uint8_t place_position[6] = {1, 2, 3, 4, 5, 6};
uint64_t system_runtime_ms = 0;
uint8_t beep_flag = 0;
uint8_t long_beep_flag = 0;
uint8_t current_step = 0;

//抓取货物动作组编号
const uint8_t get_goods_actions[6]={1,2,3,4,5,6};
//举起货物动作组编号
const uint8_t lift_goods_actions[6]={7,8,9,10,11,12};
//放置货物动作组编号
const uint8_t place_goods_actions[6]={13,14,15,16,17,18};
//单抓 + 举起货物动作组编号
const uint8_t get_and_lift_goods_actions[6]={19,20,21,22,23,24};
//每组抓取动作执行时间
uint16_t get_goods_time[6]={5300,5200,5300,5800,5800,5800};
//举起货物时间
uint16_t lift_goods_time[6]={3800,3800,3800,3800,3800,3800};
//放置货物时间
uint16_t place_goods_time[6]={4100,4100,4100,4100,4100,4100};

//单抓 + 举起货物动作时间
uint16_t get_and_lift_goods_time[6]={5000,5000,5000,5000,5000,5000};


void beep(void)
{
    static uint8_t cnt = 0;
    if (beep_flag)
    {
        cnt++;
        HAL_GPIO_TogglePin(beep_GPIO_Port, beep_Pin);
        if (cnt > 8)
        {
            cnt = 0;
            beep_flag = 0;
            beep_stop;
        }
    }
    
}

/*中值滤波*/
uint16_t median_filter(uint32_t *array, uint8_t n)
{
    uint8_t i = 0, j = 0;
    uint32_t temp = 0;
    for (i = 0; i < n - 1; i++)
    {
        for (j = 0; j < n - i; j++)
        {
            if (*(array + i) > *(array + i + 1))
            {
                temp = *(array + i);
                *(array + i) = *(array + i + 1);
                *(array + i + 1) = temp;
            }
        }
    }
    return *(array + (n - 1) / 2);
}
float normalization(float value, float min, float max, float normalization_min, float normalization_max)
{
    float temp;
    temp = value;
    temp = temp < min ? min : temp;
    temp = temp > max ? max : temp;
    if (max - min > 1.0f)
    {
        temp = (temp - min) / (max - min) * (normalization_max - normalization_min) + normalization_min;
    }
    else
    {
        return normalization_min;
    }
    return temp;
}

pid_type pid;
float ad1_nor, ad2_nor, ad3_nor;

float ad1_min = 2000, ad1_max = 0;
float ad2_min = 2000, ad2_max = 0;
float ad3_min = 2000, ad3_max = 0;

void environment_collect(void)
{
    uint32_t ad1_tmp[21], ad2_tmp[21], ad3_tmp[21];
    uint32_t tmp1, tmp2, tmp3;
    if (HAL_GPIO_ReadPin(collect_GPIO_Port, collect_Pin) == GPIO_PIN_RESET)
    {

        HAL_Delay(50);
        if (HAL_GPIO_ReadPin(collect_GPIO_Port, collect_Pin) == GPIO_PIN_RESET)
        {
            ad1_min = ad2_min = ad3_min = 2000;
            ad1_max = ad2_max = ad3_max = 0;
            while (HAL_GPIO_ReadPin(collect_GPIO_Port, collect_Pin) == GPIO_PIN_RESET)
                ;
            while (HAL_GPIO_ReadPin(collect_GPIO_Port, collect_Pin) == GPIO_PIN_RESET)
                ;
            while (1)
            {
                for (uint8_t i = 0; i < 21; i++)
                {
                    ad1_tmp[i] = adc_values[0];
                    ad2_tmp[i] = adc_values[1];
                    ad3_tmp[i] = adc_values[2];
                    HAL_Delay(5);
                }
                tmp1 = median_filter(ad1_tmp, 21);
                tmp2 = median_filter(ad2_tmp, 21);
                tmp3 = median_filter(ad3_tmp, 21);
                if (ad1_min > tmp1 * 1.0f)
                {
                    ad1_min = tmp1 * 1.0f;
                }
                if (ad2_min > tmp2 * 1.0f)
                {
                    ad2_min = tmp2 * 1.0f;
                }
                if (ad3_min > tmp3 * 1.0f)
                {
                    ad3_min = tmp3 * 1.0f;
                }
                
                if (ad1_max < tmp1 * 0.834f)
                {
                    ad1_max = tmp1 * 1.2f;
                }
                if (ad2_max < tmp2 * 0.834f)
                {
                    ad2_max = tmp2 * 1.2f;
                }
                if (ad3_max < tmp3 * 0.834f)
                {
                    ad3_max = tmp3 * 1.2f;
                }
                oled_printf(0, 2, "ad   nor  min   max");

                oled_printf(0, 3, "%d", adc_values[0]);
                oled_printf(0, 4, "%d", adc_values[1]);
                oled_printf(0, 5, "%d", adc_values[2]);

                oled_printf(5, 3, "%.0f", ad1_nor);
                oled_printf(5, 4, "%.0f", ad2_nor);
                oled_printf(5, 5, "%.0f", ad3_nor);

                oled_printf(9, 3, " %.0f", ad1_min);
                oled_printf(9, 4, " %.0f", ad2_min);
                oled_printf(9, 5, " %.0f", ad3_min);

                oled_printf(15, 3, " %.0f", ad1_max);
                oled_printf(15, 4, " %.0f", ad2_max);
                oled_printf(15, 5, " %.0f", ad3_max);
                OLED_Updata();
                OLED_ClearScreen();
                if (HAL_GPIO_ReadPin(collect_GPIO_Port, collect_Pin) == GPIO_PIN_RESET)
                {
                    HAL_Delay(50);
                    if (HAL_GPIO_ReadPin(collect_GPIO_Port, collect_Pin) == GPIO_PIN_RESET)
                    {
                        save_flag = 1;
                        while (HAL_GPIO_ReadPin(collect_GPIO_Port, collect_Pin) == GPIO_PIN_RESET)
                            ;
                        while (HAL_GPIO_ReadPin(collect_GPIO_Port, collect_Pin) == GPIO_PIN_RESET)
                            ;
                        beep_start;
                        HAL_Delay(50);
                        beep_stop;
                        return;
                    }
                }
            }
        }
    }
}

void err_calc(void)
{
    ad1_nor = normalization(adc_values[0] * 1.0, 100.0f, 2000.0f, 1.0f, 10.0f);
    ad3_nor = normalization(adc_values[2] * 1.0, 100.0f, 2000.0f, 1.0f, 10.0f);
    ad2_nor = normalization(adc_values[1] * 1.0, 100.0f, 2000.0f, 1.0f, 10.0f);
    pid.err = (ad1_nor - ad3_nor) / (ad1_nor + ad3_nor);
}
void pid_calc()
{
    pid.delta_err = pid.err - pid.last_err;
    pid.last_err = pid.err;
    if (pid.err > 0)
        pid.pidout = pid.kp_l * pid.err + pid.kd_l * pid.delta_err;
    else
        pid.pidout = pid.kp_r * pid.err + pid.kd_r * pid.delta_err;
}

#define myABS(n) ((n >= 0) ? (n) : (-(n)))
int16_t left_speed, right_speed;
void motor(int16_t speed_l, int16_t speed_r, uint8_t is_turn)
{
    static uint8_t cnt = 0;

    //is_turn = 0时 只允许前进
    if (!is_turn)
    {
        if (speed_l < 0)
            speed_l = 0;
        if (speed_r < 0)
            speed_r = 0;
    }
    if(speed_l > 1000)
    {
        speed_r -= (speed_l-1000);
        speed_l = 1000;
    }
    if(speed_r > 1000)
    {
        speed_l -= (speed_r-1000);
        speed_r = 1000;
    }
    
    if (speed_l >= 0)
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1000);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1000 - speed_l);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1000 - speed_l);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 1000);
        
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 1000 - myABS(speed_l));
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 1000);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, 1000);
        __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, 1000 - myABS(speed_l));
        
    }
    if (speed_r >= 0)
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000 - speed_r);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000 - speed_r);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);
        
    }
    else
    {
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000 - myABS(speed_r));
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000 - myABS(speed_r));
        __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000);
    }
    left_speed = speed_l;
    right_speed = speed_r;
}
int16_t speed = 0;

uint8_t is_right_sensor_valid = 0;
uint8_t is_head_right_sensor_valid = 0;
uint8_t is_head_left_sensor_valid = 0;
uint8_t is_IR_sensor_valid = 0;
uint8_t is_left_left_valid = 0;
uint8_t is_head1_valid = 0;
uint8_t is_head2_valid = 0;
void read_sensor()
{
    static uint8_t cnt1=0,cnt2=0,cnt3=0,cnt4=0,cnt5=0,cnt6=0,cnt7=0;
    
    if(!is_right_sensor_valid && HAL_GPIO_ReadPin(right_sensor_GPIO_Port, right_sensor_Pin) == GPIO_PIN_SET)
        cnt1 ++;
    if(is_right_sensor_valid && HAL_GPIO_ReadPin(right_sensor_GPIO_Port, right_sensor_Pin) == GPIO_PIN_RESET)
        cnt1 ++;
    if(cnt1 >= 2)
    {
        cnt1 = 0;
        is_right_sensor_valid = 1 - is_right_sensor_valid;
    }
    
    if(!is_head_right_sensor_valid && HAL_GPIO_ReadPin(head_right_GPIO_Port, head_right_Pin) == GPIO_PIN_SET)
        cnt2 ++;
    if(is_head_right_sensor_valid && HAL_GPIO_ReadPin(head_right_GPIO_Port, head_right_Pin) == GPIO_PIN_RESET)
        cnt2 ++;
    if(cnt2 >= 2)
    {
        cnt2 = 0;
        is_head_right_sensor_valid = 1 - is_head_right_sensor_valid;
    }
    
    if(!is_head_left_sensor_valid && HAL_GPIO_ReadPin(head_left_GPIO_Port, head_left_Pin) == GPIO_PIN_SET)
        cnt3 ++;
    if(is_head_left_sensor_valid && HAL_GPIO_ReadPin(head_left_GPIO_Port, head_left_Pin) == GPIO_PIN_RESET)
        cnt3 ++;
    if(cnt3 >= 2)
    {
        cnt3 = 0;
        is_head_left_sensor_valid = 1 - is_head_left_sensor_valid;
    }
    
    
    
    if(!is_IR_sensor_valid && HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) == GPIO_PIN_RESET)
        cnt4 ++;
    if(is_IR_sensor_valid && HAL_GPIO_ReadPin(IR_GPIO_Port, IR_Pin) == GPIO_PIN_SET)
        cnt4 ++;
    if(cnt4 >= 2)
    {
        cnt4 = 0;
        is_IR_sensor_valid = 1 - is_IR_sensor_valid;
    }
    
    if(!is_left_left_valid && HAL_GPIO_ReadPin(head_mostleft_GPIO_Port, head_mostleft_Pin) == GPIO_PIN_SET)
        cnt5 ++;
    if(is_left_left_valid && HAL_GPIO_ReadPin(head_mostleft_GPIO_Port, head_mostleft_Pin) == GPIO_PIN_RESET)
        cnt5 ++;
    if(cnt5 >= 2)
    {
        cnt5 = 0;
        is_left_left_valid = 1 - is_left_left_valid;
    }
    
    if(!is_head1_valid && HAL_GPIO_ReadPin(head1_GPIO_Port, head1_Pin) == GPIO_PIN_SET)
        cnt6 ++;
    if(is_head1_valid && HAL_GPIO_ReadPin(head1_GPIO_Port, head1_Pin) == GPIO_PIN_RESET)
        cnt6 ++;
    if(cnt6 >= 3)
    {
        cnt6 = 0;
        is_head1_valid = 1 - is_head1_valid;
    }
    
    if(!is_head2_valid && HAL_GPIO_ReadPin(head2_GPIO_Port, head2_Pin) == GPIO_PIN_SET)
        cnt7 ++;
    if(is_head2_valid && HAL_GPIO_ReadPin(head2_GPIO_Port, head2_Pin) == GPIO_PIN_RESET)
        cnt7 ++;
    if(cnt7 >= 2)
    {
        cnt7 = 0;
        is_head2_valid = 1 - is_head2_valid;
    }
    
    
}

void baffle_first_stop(uint8_t mode)
{
    static int16_t spd = 9999;
    if(mode == 0)
    {
        spd = 9999;
    }
    else
    {
        if(spd > baffle_speed)
            spd = baffle_speed;
        spd -= 25;
        if(spd > 0)
            motor(spd, (spd), 0);
        else
            motor(0, 0, 0);
    }
}
void baffle_second_stop(uint8_t mode)
{
    static int16_t spd = 9999;
    if(mode == 0)
    {
        spd = 9999;
    }
    else
    {
        if(spd > baffle_speed)
            spd = baffle_speed;
        spd -= baffle_speed/10;
        if(spd > 0)
            motor(spd, (spd), 0);
        else
            motor(0, 0, 0);
    }
}

void run()
{
    static uint64_t go_straight_time = 0;
    static uint8_t is_slow = 0;
    static uint8_t last_head1 = 0;
    pid_calc();
    if (pid.pidout > control.speed)
        pid.pidout = control.speed;
    if (pid.pidout < -control.speed)
        pid.pidout = -control.speed;
    if (speed < control.speed)
        speed += 5;
    else if (speed >= control.speed)
        speed = control.speed;
    
    //头部2个传感器
    if ((ad2_nor >= 4))
    {
        if(current_step <= first_station && is_head1_valid)
        {
            is_slow = 1;
        }
        if(current_step == baffle)
        {
            is_slow = is_head2_valid;
            if(is_head2_valid)
            {
                go_straight_time = system_runtime_ms;
            }
        }
        if(current_step == uphill_road && is_head1_valid)
        {
            is_slow = 1;
        }
        if(is_right_sensor_valid)
        {
            is_slow = 0;
        }
    }
    last_head1 = is_head2_valid;
    if(is_slow)
    {
        //设置速度
        if(current_step < uphill_road)
        {
            speed = first_station_stop_speed;
        }
        else
        {
            speed = second_station_speed;
        }
    }
    
    //压在线上或者传感器越过横向黑线（防止被黑线干扰循迹）则正常循迹,
    if (!is_slow && go_straight_time + 300 < system_runtime_ms && (ad1_nor > 2 || ad2_nor > 2 || ad3_nor > 2 || is_head_left_sensor_valid || is_head_right_sensor_valid))
    {
        beep_stop;
        //巡线左侧传感器
        if (is_head_left_sensor_valid && (ad1_nor <= 2 && ad2_nor <= 2 && ad3_nor <= 2 ))
        {
            motor(0, 1000, 0);
            return;
        }
        if(current_step <= first_station)
        {
            if (is_left_left_valid && (ad1_nor <= 2 && ad2_nor <= 2 && ad3_nor <= 2 ))
            {
                motor(0, 1000, 0);
                return;
            }
        }
        //巡线右侧传感器
        if (is_head_right_sensor_valid && (ad1_nor <= 2 && ad2_nor <= 2 && ad3_nor <= 2 ))
        {
            motor(1000, 0, 0);
            return;
        }
        //正常循迹
        motor(speed - pid.pidout, speed + pid.pidout, 0);
    }
    //丢线直走
    else
    {
        beep_start;
        if(current_step < downhill_road)
        {
            motor(speed, speed, 0);
        }
        else
        {
            if(control.right_wheel_speed_offset > 0)
                motor(speed, (speed + control.right_wheel_speed_offset), 0);
            else
                motor(speed - control.right_wheel_speed_offset, speed, 0);
        }
    }
}




uint8_t action_num = 100;


void work(void)
{
    static int16_t saved_speed;
    static uint8_t cnt = 0;
    static uint64_t turn_time = 0;
    static uint8_t goods_num = 0;
    if (control.run_flag)
    {
        switch (control.run_mode)
        {
        //循迹至取物台
        case 0:
            unfixed;
            if(control.task < 2)
            {
                set_run_mode(1);
            }
            else
            {
                set_run_mode(50);
            }
            break;
        case 1:
            run();
            if (is_time_out_ms(1000))
            {
                next_run_mode;
            }
            break;
        case 2:
            run();
            if (is_right_sensor_valid)
            {
                current_step = first_station;
                next_run_mode;
            }
            break;
        case 3:
            //停车2秒 抓下排红
            if(cnt < 5)
            {
                action_num = get_goods_actions[0];
                cnt = 10;
            }
            if (is_time_out_ms(get_goods_time[0]))
            {
                cnt = 0;
                next_run_mode;
            }
            break;
        case 4:
            //停车2秒 抓下排蓝
            if(cnt < 5)
            {
                action_num = get_goods_actions[1];
                cnt = 10;
            }
            if (is_time_out_ms(get_goods_time[1]))
            {
                cnt = 0;
                next_run_mode;
            }
            break;
        case 5:
            //停车2秒 抓下排绿
            if(cnt < 5)
            {
                action_num = get_goods_actions[2];
                cnt = 10;
            }
            if (is_time_out_ms(get_goods_time[2]))
            {
                cnt = 0;
                if(control.task == 1)
                {
                    next_run_mode;
                }
                else
                {
                    fixed;
                    cnt = 0;
                    //机械手归位
                    action_num = 0;
                    set_run_mode(9);
                }
            }
            break;
        case 6:
            //停车2秒 抓上排红
            if(cnt < 5)
            {
                action_num = get_goods_actions[3];
                cnt = 10;
            }
            if (is_time_out_ms(get_goods_time[3]))
            {
                cnt = 0;
                next_run_mode;
            }
            break;
        case 7:
            //停车2秒 抓上排蓝
            if(cnt < 5)
            {
                action_num = get_goods_actions[4];
                cnt = 10;
            }
            if (is_time_out_ms(get_goods_time[4]))
            {
                cnt = 0;
                next_run_mode;
            }
            break;
        case 8:
            //停车2秒 抓上排绿
            if(cnt < 5)
            {
                action_num = get_goods_actions[5];
                cnt = 10;
            }
            if (is_time_out_ms(get_goods_time[5]))
            {
                fixed;
                cnt = 0;
                //机械手归位
                action_num = 0;
                next_run_mode;
            }
            break;
            
        case 9:
            if (is_time_out_ms(300))
            {
                unfixed;
                next_run_mode;
            }
            break;
        case 10:
            if (is_time_out_ms(800))
            {
                fixed;
                next_run_mode;
            }
            break;
        case 11:
            if (is_time_out_ms(400))
            {
                next_run_mode;
            }
            break;
        case 12:
            //检测挡板
            run();
            if(is_IR_sensor_valid)
            {
                current_step = baffle;
                baffle_first_stop(0);
                next_run_mode;
            }
            break;
        case 13:
            //停半秒
            baffle_first_stop(1);
            if (is_time_out_ms(400))
            {
                next_run_mode;
            }
            break;
        case 14:
            //右转一点
            robot_turn_next(turn_right_spd,turn_right_time);
            break;
        case 15:
            //停顿0.3秒
            if (is_time_out_ms(300))
            {
                next_run_mode;
            }
            break;
        case 16:
            //向前跑，直到传感器检测到线
            motor(baffle_speed,baffle_speed,0);
            if (is_right_sensor_valid)
            {
                baffle_second_stop(0);
                next_run_mode;
            }
            break;
        case 17:
            //停顿半秒
            baffle_second_stop(1);
            if (is_time_out_ms(300))
            {
                next_run_mode;
            }
            break;
        case 18:
            //左转一点
            robot_turn_next(-turn_left_spd,turn_left_time);
            break;
        case 19:
            //继续循迹至坡道
            run();
            //如果最左侧灰度检测到黑线加速冲坡
            if (is_left_left_valid)
            {
                next_run_mode;
            }
            break;
        case 20:
            run();
            //检测到坡道右侧黑线，恢复正常速度
            if(is_head_right_sensor_valid)
            {
                control.speed = saved_speed;
                next_run_mode;
            }
            if(!is_left_left_valid && current_step < uphill_road)
            {
                saved_speed = control.speed;
                control.speed = uphill_speed;
                current_step = uphill_road;
            }
            break;
        case 21:
            //循迹，直到对准物料台
            run();
            if(is_right_sensor_valid)
            {
                unfixed;
                current_step = second_station;
                next_run_mode;
            }
            break;
        case 22:
            //举起物料1
            if(cnt < 5)
            {
                action_num = lift_goods_actions[0];
                cnt = 10;
            }
            if (is_time_out_ms(lift_goods_time[0]))
            {
                cnt = 0;
                current_step = downhill_road;
                next_run_mode;
            }
            break;
        case 23:
            //放置物料1
            if(cnt < 5)
            {
                action_num = place_goods_actions[place_position[0] - 1];
                cnt = 10;
            }
            if (is_time_out_ms(place_goods_time[place_position[0] - 1]))
            {
                cnt = 0;
                next_run_mode;
            }
            break;
            
            
        case 24:
            //举起物料2
            if(cnt < 5)
            {
                action_num = lift_goods_actions[1];
                cnt = 10;
            }
            if (is_time_out_ms(lift_goods_time[1]))
            {
                cnt = 0;
                next_run_mode;
            }
            break;
        case 25:
            //放置物料2
            if(cnt < 5)
            {
                action_num = place_goods_actions[place_position[1] - 1];
                cnt = 10;
            }
            if (is_time_out_ms(place_goods_time[place_position[1] - 1]))
            {
                cnt = 0;
                next_run_mode;
            }
            break;
        case 26:
            //举起物料3
            if(cnt < 5)
            {
                action_num = lift_goods_actions[2];
                cnt = 10;
            }
            if (is_time_out_ms(lift_goods_time[2]))
            {
                cnt = 0;
                next_run_mode;
            }
            break;
        case 27:
            //放置物料3
            if(cnt < 5)
            {
                action_num = place_goods_actions[place_position[2] - 1];
                cnt = 10;
            }
            if (is_time_out_ms(place_goods_time[place_position[2] - 1]))
            {
                cnt = 0;
                if(control.task == 0)
                {
                    set_run_mode(31);
                }
                else
                {
                    next_run_mode;
                }
            }
            break;
        case 28:
            //举起物料4
            if(cnt < 5)
            {
                action_num = lift_goods_actions[3];
                cnt = 10;
            }
            if (is_time_out_ms(lift_goods_time[3]))
            {
                cnt = 0;
                next_run_mode;
            }
            break;
        case 29:
            //放置物料4
            if(cnt < 5)
            {
                action_num = place_goods_actions[place_position[3] - 1];
                cnt = 10;
            }
            if (is_time_out_ms(place_goods_time[place_position[3] - 1]))
            {
                cnt = 0;
                next_run_mode;
            }
            break;
        case 30:
            //举起物料5
            if(cnt < 5)
            {
                action_num = lift_goods_actions[4];
                cnt = 10;
            }
            if (is_time_out_ms(lift_goods_time[4]))
            {
                cnt = 0;
                next_run_mode;
            }
            break;
            
            
            
        case 31:
            //放置物料5
            if(cnt < 5)
            {
                action_num = place_goods_actions[place_position[4] - 1];
                cnt = 10;
            }
            if (is_time_out_ms(place_goods_time[place_position[4] - 1]))
            {
                cnt = 0;
                next_run_mode;
            }
            break;
        case 32:
            //举起物料6
            if(cnt < 5)
            {
                action_num = lift_goods_actions[5];
                cnt = 10;
            }
            if (is_time_out_ms(lift_goods_time[5]))
            {
                cnt = 0;
                next_run_mode;
            }
            break;
        case 33:
            //放置物料6
            if(cnt < 5)
            {
                action_num = place_goods_actions[place_position[5] - 1];
                cnt = 10;
            }
            if (is_time_out_ms(place_goods_time[place_position[5] - 1]))
            {
                cnt = 0;
                //机械手归位
                action_num = 0;
                next_run_mode;
            }
            break;
        case 34:
            //抓完了，下坡循迹2.5s
            run();
            if (is_time_out_ms(1500))
            {
                next_run_mode;
            }
            break;
        case 35:
            //最后一个弯停顿一下
            run();
            if (!(ad1_nor > 2 || ad2_nor > 2 || ad3_nor > 2 || is_head_left_sensor_valid || is_head_right_sensor_valid))
            {
                next_run_mode;
            }
            break;
        case 36:
            run();
            if (is_time_out_ms(400))
            {
                beep_stop;
                next_run_mode;
            }
            break;
        case 37:
            break;
        case 38:
            break;
        
        
        
        
        
        //***************************************************
        //***************************************************
        //*****************    单抓单跑    *****************
        //***************************************************
        //***************************************************
        
        
        
        case 50:
            run();
            if (is_time_out_ms(1000))
            {
                next_run_mode;
            }
            break;
        case 51:
            run();
            if (is_right_sensor_valid)
            {
                current_step = first_station;
                next_run_mode;
            }
            break;
        case 52:
            //停车2秒 抓下排红
            action_num = get_and_lift_goods_actions[goods_num];
            if (is_time_out_ms(get_and_lift_goods_time[goods_num]))
            {
                next_run_mode;
            }
            break;
        case 53:
            //检测挡板
            run();
            if(is_IR_sensor_valid)
            {
                current_step = baffle;
                baffle_first_stop(0);
                next_run_mode;
            }
            break;
        case 54:
            //停半秒
            baffle_first_stop(1);
            if (is_time_out_ms(400))
            {
                next_run_mode;
            }
            break;
        case 55:
            //右转一点
            robot_turn_next(800,turn_right_time);
            break;
        case 56:
            //停顿0.3秒
            if (is_time_out_ms(300))
            {
                next_run_mode;
            }
            break;
        case 57:
            //向前跑，直到传感器检测到线
            motor(baffle_speed,baffle_speed,0);
            if (is_right_sensor_valid)
            {
                baffle_second_stop(0);
                next_run_mode;
            }
            break;
        case 58:
            //停顿半秒
            baffle_second_stop(1);
            if (is_time_out_ms(300))
            {
                next_run_mode;
            }
            break;
        case 59:
            //左转一点
            robot_turn_next(-600,200);
            break;
        case 60:
            //继续循迹至坡道
            run();
            //如果最左侧灰度检测到黑线加速冲坡
            if (is_left_left_valid)
            {
                next_run_mode;
            }
            break;
        case 61:
            run();
            //检测到坡道右侧黑线，恢复正常速度
            if(is_head_right_sensor_valid)
            {
                control.speed = saved_speed;
                next_run_mode;
            }
            if(!is_left_left_valid && current_step < uphill_road)
            {
                saved_speed = control.speed;
                control.speed = uphill_speed;
                current_step = uphill_road;
            }
            break;
        case 62:
            //循迹，直到对准物料台
            run();
            if(is_right_sensor_valid)
            {
                current_step = second_station;
                next_run_mode;
            }
            break;
        case 63:
            //放置物料
            action_num = lift_goods_actions[place_position[goods_num] - 1];
            if (is_time_out_ms(lift_goods_time[place_position[goods_num] - 1]))
            {
                current_step = downhill_road;
                next_run_mode;
            }
            break;
        case 64:
            //机械手归位
            action_num = 0;
            //抓完了，下坡循迹2.5s
            run();
            if (is_time_out_ms(1500))
            {
                next_run_mode;
            }
            break;
        case 65:
            //最后一个弯停顿一下
            run();
            if (!(ad1_nor > 2 || ad2_nor > 2 || ad3_nor > 2 || is_head_left_sensor_valid || is_head_right_sensor_valid))
            {
                next_run_mode;
            }
            break;
        case 66:
            run();
            if (is_time_out_ms(400))
            {
                next_run_mode;
            }
            break;
        case 67:
            beep_stop;
            current_step = 0;
            goods_num++;
            if(goods_num < 6)
            {
                set_run_mode(50);
            }
            else
            {
                next_run_mode;
            }
            break;
        case 68:
            break;
        
        
        
        
        }
    }
    else
    {
        stop_motor;
    }
}
