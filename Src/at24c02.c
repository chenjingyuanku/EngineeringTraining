#include "stm32f1xx_hal.h"
#include "at24c02.h"
#include "control.h"
#include "ui.h"
#define FLASH_WriteAddress 0x803F000//0x8070000
#define FLASH_ReadAddress FLASH_WriteAddress
#define FLASH_TESTSIZE 128
#define STM32_FLASH_SIZE 512 // 所选STM32的FLASH容量大小(单位为K)
uint16_t Tx_Buffer[FLASH_TESTSIZE] = {0};
uint16_t Rx_Buffer[FLASH_TESTSIZE] = {0};

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
#if STM32_FLASH_SIZE < 256
#define STM_SECTOR_SIZE 1024 //字节
#else
#define STM_SECTOR_SIZE 2048
#endif

/* 私有变量 ------------------------------------------------------------------*/
static uint16_t STMFLASH_BUF[STM_SECTOR_SIZE / 2]; //最多是2K字节
static FLASH_EraseInitTypeDef EraseInitStruct;

/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 读取指定地址的半字(16位数据)
  * 输入参数: faddr:读地址(此地址必须为2的倍数!!)
  * 返 回 值: 返回值:对应数据.
  * 说    明：无
  */
uint16_t STMFLASH_ReadHalfWord(uint32_t faddr)
{
    return *(__IO uint16_t *)faddr;
}

/**
  * 函数功能: 不检查的写入
  * 输入参数: WriteAddr:起始地址
  *           pBuffer:数据指针
  *           NumToWrite:半字(16位)数
  * 返 回 值: 无
  * 说    明：无
  */
void STMFLASH_Write_NoCheck(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite)
{
    uint16_t i;

    for (i = 0; i < NumToWrite; i++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, WriteAddr, pBuffer[i]);
        WriteAddr += 2; //地址增加2.
    }
}

/**
  * 函数功能: 从指定地址开始读出指定长度的数据
  * 输入参数: ReadAddr:起始地址
  *           pBuffer:数据指针
  *           NumToRead:半字(16位)数
  * 返 回 值: 无
  * 说    明：无
  */
void STMFLASH_Read(uint32_t ReadAddr, uint16_t *pBuffer, uint16_t NumToRead)
{
    uint16_t i;

    for (i = 0; i < NumToRead; i++)
    {
        pBuffer[i] = STMFLASH_ReadHalfWord(ReadAddr); //读取2个字节.
        ReadAddr += 2;                                //偏移2个字节.
    }
}

/**
  * 函数功能: 从指定地址开始写入指定长度的数据
  * 输入参数: WriteAddr:起始地址(此地址必须为2的倍数!!)
  *           pBuffer:数据指针
  *           NumToWrite:半字(16位)数(就是要写入的16位数据的个数.)
  * 返 回 值: 无
  * 说    明：无
  */
void STMFLASH_Write(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite)
{
    uint32_t SECTORError = 0;
    uint16_t secoff;    //扇区内偏移地址(16位字计算)
    uint16_t secremain; //扇区内剩余地址(16位字计算)
    uint16_t i;
    uint32_t secpos;  //扇区地址
    uint32_t offaddr; //去掉0X08000000后的地址

    if (WriteAddr < FLASH_BASE || (WriteAddr >= (FLASH_BASE + 1024 * STM32_FLASH_SIZE)))
        return; //非法地址

    HAL_FLASH_Unlock(); //解锁

    offaddr = WriteAddr - FLASH_BASE;         //实际偏移地址.
    secpos = offaddr / STM_SECTOR_SIZE;       //扇区地址  0~127 for STM32F103RBT6
    secoff = (offaddr % STM_SECTOR_SIZE) / 2; //在扇区内的偏移(2个字节为基本单位.)
    secremain = STM_SECTOR_SIZE / 2 - secoff; //扇区剩余空间大小
    if (NumToWrite <= secremain)
        secremain = NumToWrite; //不大于该扇区范围

    while (1)
    {
        STMFLASH_Read(secpos * STM_SECTOR_SIZE + FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE / 2); //读出整个扇区的内容
        for (i = 0; i < secremain; i++)                                                          //校验数据
        {
            if (STMFLASH_BUF[secoff + i] != 0XFFFF)
                break; //需要擦除
        }
        if (i < secremain) //需要擦除
        {
            //擦除这个扇区
            /* Fill EraseInit structure*/
            EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
            EraseInitStruct.PageAddress = secpos * STM_SECTOR_SIZE + FLASH_BASE;
            EraseInitStruct.NbPages = 1;
            HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError);

            for (i = 0; i < secremain; i++) //复制
            {
                STMFLASH_BUF[i + secoff] = pBuffer[i];
            }
            STMFLASH_Write_NoCheck(secpos * STM_SECTOR_SIZE + FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE / 2); //写入整个扇区
        }
        else
        {
            STMFLASH_Write_NoCheck(WriteAddr, pBuffer, secremain); //写已经擦除了的,直接写入扇区剩余区间.
        }
        if (NumToWrite == secremain)
            break; //写入结束了
        else       //写入未结束
        {
            secpos++;                //扇区地址增1
            secoff = 0;              //偏移位置为0
            pBuffer += secremain;    //指针偏移
            WriteAddr += secremain;  //写地址偏移
            NumToWrite -= secremain; //字节(16位)数递减
            if (NumToWrite > (STM_SECTOR_SIZE / 2))
                secremain = STM_SECTOR_SIZE / 2; //下一个扇区还是写不完
            else
                secremain = NumToWrite; //下一个扇区可以写完了
        }
    };
    HAL_FLASH_Lock(); //上锁
}
typedef union{
    uint8_t u8[2];
    uint16_t u16;
    int16_t s16;
}b16;
typedef union{
    float ft;
    uint8_t u8[4];
}b32;


uint8_t save_flag = 0;
uint8_t save_show_flag = 0;
void write_float_to_tx_buf(float ft,uint8_t index)
{
    b32 bb;
    bb.ft = ft;
    Tx_Buffer[index]     = bb.u8[0];
    Tx_Buffer[index + 1] = bb.u8[1];
    Tx_Buffer[index + 2] = bb.u8[2];
    Tx_Buffer[index + 3] = bb.u8[3];
}
void read_float_from_rx_buf(float *ft,uint8_t index)
{
    b32 bb;
    bb.u8[0] = Rx_Buffer[index];
    bb.u8[1] = Rx_Buffer[index + 1];
    bb.u8[2] = Rx_Buffer[index + 2];
    bb.u8[3] = Rx_Buffer[index + 3];
    *ft = bb.ft;
}
void write_u16_to_tx_buf(uint16_t u16,uint8_t index)
{
    b16 b;
    b.u16 = u16;
    Tx_Buffer[index] = b.u8[0];
    Tx_Buffer[index + 1] = b.u8[1];
}
void write_s16_to_tx_buf(int16_t s16,uint8_t index)
{
    b16 b;
    b.s16 = s16;
    Tx_Buffer[index] = b.u8[0];
    Tx_Buffer[index + 1] = b.u8[1];
}

void read_u16_to_tx_buf(uint16_t *u16,uint8_t index)
{
    b16 b;
    b.u8[0]=Rx_Buffer[index];
    b.u8[1]=Rx_Buffer[index+1];
    *u16 = b.u16;
}
void read_s16_to_tx_buf(int16_t *s16,uint8_t index)
{
    b16 b;
    b.u8[0]=Rx_Buffer[index];
    b.u8[1]=Rx_Buffer[index+1];
    *s16 = b.s16;
}

void save_parama(void)
{
    b32 bb;
    if(save_flag)
    {
        for(uint8_t i=0;i<6;i++)
        {
            Tx_Buffer[i] = place_position[i];
        }
        write_s16_to_tx_buf(control.speed,6);
        write_s16_to_tx_buf(control.right_wheel_speed_offset,8);
        
        write_float_to_tx_buf(pid.kp_l,10);
        write_float_to_tx_buf(pid.kd_l,14);
        write_float_to_tx_buf(pid.kp_r,18);
        write_float_to_tx_buf(pid.kd_r,22);
        write_float_to_tx_buf(ad1_min,26);
        write_float_to_tx_buf(ad1_max,30);
        write_float_to_tx_buf(ad2_min,34);
        write_float_to_tx_buf(ad2_max,38);
        write_float_to_tx_buf(ad3_min,42);
        write_float_to_tx_buf(ad3_max,46);
        Tx_Buffer[50] = control.task;
        
        write_u16_to_tx_buf(uphill_speed,51);
        write_u16_to_tx_buf(baffle_speed,53);
        write_u16_to_tx_buf(turn_right_time,55);
        
        write_u16_to_tx_buf(turn_right_spd,57);
        write_u16_to_tx_buf(first_station_stop_speed,59);
        write_u16_to_tx_buf(turn_left_spd,61);
        write_u16_to_tx_buf(second_station_speed,63);
        
        
        write_u16_to_tx_buf(get_goods_time[0],65);
        write_u16_to_tx_buf(get_goods_time[1],67);
        write_u16_to_tx_buf(get_goods_time[2],69);
        write_u16_to_tx_buf(get_goods_time[3],71);
        write_u16_to_tx_buf(get_goods_time[4],73);
        write_u16_to_tx_buf(get_goods_time[5],75);
        
        write_u16_to_tx_buf(lift_goods_time[0],77);
        write_u16_to_tx_buf(lift_goods_time[1],79);
        write_u16_to_tx_buf(lift_goods_time[2],81);
        write_u16_to_tx_buf(lift_goods_time[3],83);
        write_u16_to_tx_buf(lift_goods_time[4],85);
        write_u16_to_tx_buf(lift_goods_time[5],87);
        
        write_u16_to_tx_buf(place_goods_time[0],89);
        write_u16_to_tx_buf(place_goods_time[1],91);
        write_u16_to_tx_buf(place_goods_time[2],93);
        write_u16_to_tx_buf(place_goods_time[3],95);
        write_u16_to_tx_buf(place_goods_time[4],97);
        write_u16_to_tx_buf(place_goods_time[5],99);
        
        write_u16_to_tx_buf(get_and_lift_goods_time[0],101);
        write_u16_to_tx_buf(get_and_lift_goods_time[1],103);
        write_u16_to_tx_buf(get_and_lift_goods_time[2],105);
        write_u16_to_tx_buf(get_and_lift_goods_time[3],107);
        write_u16_to_tx_buf(get_and_lift_goods_time[4],109);
        write_u16_to_tx_buf(get_and_lift_goods_time[5],111);
        
        Tx_Buffer[113] = place_position_task0[0];
        Tx_Buffer[114] = place_position_task0[1];
        Tx_Buffer[115] = place_position_task0[2];
        
        write_u16_to_tx_buf(down_unfixed_duty,116);
        write_u16_to_tx_buf(up_unfixed_duty,118);
        write_u16_to_tx_buf(down_fixed_duty,120);
        write_u16_to_tx_buf(up_fixed_duty,122);
        
        
        STMFLASH_Write(FLASH_WriteAddress, Tx_Buffer, FLASH_TESTSIZE);
        save_flag = 0;
        save_show_flag = 1;
    }
}

void read_parama(void)
{
    b16 b;
    b32 bb;
    STMFLASH_Read(FLASH_ReadAddress, Rx_Buffer, FLASH_TESTSIZE);
    HAL_Delay(50);
    for(uint8_t i=0;i<6;i++)
    {
        place_position[i] = Rx_Buffer[i];
    }
    read_s16_to_tx_buf(&control.speed,6);
    read_s16_to_tx_buf(&control.right_wheel_speed_offset,8);
    
    read_float_from_rx_buf(&pid.kp_l,10);
    read_float_from_rx_buf(&pid.kd_l,14);
    read_float_from_rx_buf(&pid.kp_r,18);
    read_float_from_rx_buf(&pid.kd_r,22);
    read_float_from_rx_buf(&ad1_min,26);
    read_float_from_rx_buf(&ad1_max,30);
    read_float_from_rx_buf(&ad2_min,34);
    read_float_from_rx_buf(&ad2_max,38);
    read_float_from_rx_buf(&ad3_min,42);
    read_float_from_rx_buf(&ad3_max,46);
    control.task = Rx_Buffer[50];
    
    
    read_u16_to_tx_buf(&uphill_speed,51);
    read_u16_to_tx_buf(&baffle_speed,53);
    read_u16_to_tx_buf(&turn_right_time,55);
    
    read_u16_to_tx_buf(&turn_right_spd,57);
    read_u16_to_tx_buf(&first_station_stop_speed,59);
    read_u16_to_tx_buf(&turn_left_spd,61);
    read_u16_to_tx_buf(&second_station_speed,63);
        
        
    read_u16_to_tx_buf(&get_goods_time[0],65);
    read_u16_to_tx_buf(&get_goods_time[1],67);
    read_u16_to_tx_buf(&get_goods_time[2],69);
    read_u16_to_tx_buf(&get_goods_time[3],71);
    read_u16_to_tx_buf(&get_goods_time[4],73);
    read_u16_to_tx_buf(&get_goods_time[5],75);
    
    read_u16_to_tx_buf(&lift_goods_time[0],77);
    read_u16_to_tx_buf(&lift_goods_time[1],79);
    read_u16_to_tx_buf(&lift_goods_time[2],81);
    read_u16_to_tx_buf(&lift_goods_time[3],83);
    read_u16_to_tx_buf(&lift_goods_time[4],85);
    read_u16_to_tx_buf(&lift_goods_time[5],87);
    
    read_u16_to_tx_buf(&place_goods_time[0],89);
    read_u16_to_tx_buf(&place_goods_time[1],91);
    read_u16_to_tx_buf(&place_goods_time[2],93);
    read_u16_to_tx_buf(&place_goods_time[3],95);
    read_u16_to_tx_buf(&place_goods_time[4],97);
    read_u16_to_tx_buf(&place_goods_time[5],99);
    
    read_u16_to_tx_buf(&get_and_lift_goods_time[0],101);
    read_u16_to_tx_buf(&get_and_lift_goods_time[1],103);
    read_u16_to_tx_buf(&get_and_lift_goods_time[2],105);
    read_u16_to_tx_buf(&get_and_lift_goods_time[3],107);
    read_u16_to_tx_buf(&get_and_lift_goods_time[4],109);
    read_u16_to_tx_buf(&get_and_lift_goods_time[5],111);
    
    
    place_position_task0[0] = Rx_Buffer[113];
    place_position_task0[1] = Rx_Buffer[114];
    place_position_task0[2] = Rx_Buffer[115];
        
    read_u16_to_tx_buf(&down_unfixed_duty,116);
    read_u16_to_tx_buf(&up_unfixed_duty,118);
    read_u16_to_tx_buf(&down_fixed_duty,120);
    read_u16_to_tx_buf(&up_fixed_duty,122);
    
}
