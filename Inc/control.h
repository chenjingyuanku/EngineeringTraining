#ifndef _control_h
#define _control_h 
#include "stm32f1xx_hal.h"

/*结构体定义*/
typedef struct
{
    uint16_t raw_value; //原始值
    uint16_t value; //值
    uint16_t max;   //最大值   
    uint16_t min;   //最小值
    float nor_value; //归一化后的值
}adc_type;


typedef struct
{
    float kp_l;
    float ki_l;
    float kd_l;
    float kp_r;
    float ki_r;
    float kd_r;
    float err;
    float last_err;
    float delta_err;
    float pidout;
    float err_variance_filtr;//方差
}pid_type;

typedef struct
{
    uint8_t run_flag;
    uint8_t run_mode;
    uint8_t page_index;
    uint8_t item_index;
    int16_t speed;
    int16_t right_wheel_speed_offset;
    uint8_t task;
}control_type;




extern uint8_t place_position[6];
extern uint8_t place_position_task0[3];
extern control_type control;
extern pid_type pid;
extern uint64_t system_runtime_ms;
extern float ad1_nor,ad2_nor,ad3_nor;
extern float ad1_min,ad1_max ;
extern float ad2_min,ad2_max ;
extern float ad3_min,ad3_max ;
extern int16_t left_speed,right_speed;
extern uint8_t beep_flag;
extern uint8_t action_num;

extern void beep(void);
extern void environment_collect(void);
extern void err_calc(void);
extern void work(void);
extern void motor(int16_t speed_l, int16_t speed_r, uint8_t is_turn);
void left_motor(uint16_t pwm,float speed,uint8_t direction);
void right_motor(uint16_t pwm,float speed,uint8_t direction);

#define beep_start HAL_GPIO_WritePin(beep_GPIO_Port,beep_Pin,1);
#define beep_stop  HAL_GPIO_WritePin(beep_GPIO_Port,beep_Pin,0);


extern uint8_t is_right_sensor_valid;
extern uint8_t is_head_right_sensor_valid;
extern uint8_t is_head_left_sensor_valid;
extern uint8_t is_IR_sensor_valid;
extern uint8_t is_left_left_valid;
extern uint8_t is_head1_valid;
extern uint8_t is_head2_valid;

extern uint16_t uphill_speed ;
extern uint16_t baffle_speed ;
extern uint16_t turn_right_time ;
extern uint16_t turn_right_spd;
extern uint16_t turn_left_spd;
extern uint16_t second_station_speed;
extern uint16_t first_station_stop_speed;


//每组抓取动作执行时间
extern uint16_t get_goods_time[2][6];
//举起货物时间
extern uint16_t lift_goods_time[2][6];
//放置货物时间
extern uint16_t place_goods_time[2][6];
//单抓 + 举起货物动作时间
extern uint16_t get_and_lift_goods_time[6];

extern uint8_t action_speed_mode;


extern uint16_t down_unfixed_duty ;
extern uint16_t up_unfixed_duty ;
extern uint16_t down_fixed_duty ;
extern uint16_t up_fixed_duty ;

extern void read_sensor(void);

extern uint8_t current_step;
enum current_step_name{
    first_station = 1,
    baffle = 2,
    uphill_road = 3,
    second_station = 4,
    downhill_road = 5
};


#endif
