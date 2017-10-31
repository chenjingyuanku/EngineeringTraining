#include "stm32f1xx_hal.h"
#include "at24c02.h"
#include "control.h"
#include "ui.h"
#define FLASH_WriteAddress 0x803F000//0x8070000
#define FLASH_ReadAddress FLASH_WriteAddress
#define FLASH_TESTSIZE 128
#define STM32_FLASH_SIZE 512 // ��ѡSTM32��FLASH������С(��λΪK)
uint16_t Tx_Buffer[FLASH_TESTSIZE] = {0};
uint16_t Rx_Buffer[FLASH_TESTSIZE] = {0};

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
#if STM32_FLASH_SIZE < 256
#define STM_SECTOR_SIZE 1024 //�ֽ�
#else
#define STM_SECTOR_SIZE 2048
#endif

/* ˽�б��� ------------------------------------------------------------------*/
static uint16_t STMFLASH_BUF[STM_SECTOR_SIZE / 2]; //�����2K�ֽ�
static FLASH_EraseInitTypeDef EraseInitStruct;

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ��ȡָ����ַ�İ���(16λ����)
  * �������: faddr:����ַ(�˵�ַ����Ϊ2�ı���!!)
  * �� �� ֵ: ����ֵ:��Ӧ����.
  * ˵    ������
  */
uint16_t STMFLASH_ReadHalfWord(uint32_t faddr)
{
    return *(__IO uint16_t *)faddr;
}

/**
  * ��������: ������д��
  * �������: WriteAddr:��ʼ��ַ
  *           pBuffer:����ָ��
  *           NumToWrite:����(16λ)��
  * �� �� ֵ: ��
  * ˵    ������
  */
void STMFLASH_Write_NoCheck(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite)
{
    uint16_t i;

    for (i = 0; i < NumToWrite; i++)
    {
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, WriteAddr, pBuffer[i]);
        WriteAddr += 2; //��ַ����2.
    }
}

/**
  * ��������: ��ָ����ַ��ʼ����ָ�����ȵ�����
  * �������: ReadAddr:��ʼ��ַ
  *           pBuffer:����ָ��
  *           NumToRead:����(16λ)��
  * �� �� ֵ: ��
  * ˵    ������
  */
void STMFLASH_Read(uint32_t ReadAddr, uint16_t *pBuffer, uint16_t NumToRead)
{
    uint16_t i;

    for (i = 0; i < NumToRead; i++)
    {
        pBuffer[i] = STMFLASH_ReadHalfWord(ReadAddr); //��ȡ2���ֽ�.
        ReadAddr += 2;                                //ƫ��2���ֽ�.
    }
}

/**
  * ��������: ��ָ����ַ��ʼд��ָ�����ȵ�����
  * �������: WriteAddr:��ʼ��ַ(�˵�ַ����Ϊ2�ı���!!)
  *           pBuffer:����ָ��
  *           NumToWrite:����(16λ)��(����Ҫд���16λ���ݵĸ���.)
  * �� �� ֵ: ��
  * ˵    ������
  */
void STMFLASH_Write(uint32_t WriteAddr, uint16_t *pBuffer, uint16_t NumToWrite)
{
    uint32_t SECTORError = 0;
    uint16_t secoff;    //������ƫ�Ƶ�ַ(16λ�ּ���)
    uint16_t secremain; //������ʣ���ַ(16λ�ּ���)
    uint16_t i;
    uint32_t secpos;  //������ַ
    uint32_t offaddr; //ȥ��0X08000000��ĵ�ַ

    if (WriteAddr < FLASH_BASE || (WriteAddr >= (FLASH_BASE + 1024 * STM32_FLASH_SIZE)))
        return; //�Ƿ���ַ

    HAL_FLASH_Unlock(); //����

    offaddr = WriteAddr - FLASH_BASE;         //ʵ��ƫ�Ƶ�ַ.
    secpos = offaddr / STM_SECTOR_SIZE;       //������ַ  0~127 for STM32F103RBT6
    secoff = (offaddr % STM_SECTOR_SIZE) / 2; //�������ڵ�ƫ��(2���ֽ�Ϊ������λ.)
    secremain = STM_SECTOR_SIZE / 2 - secoff; //����ʣ��ռ��С
    if (NumToWrite <= secremain)
        secremain = NumToWrite; //�����ڸ�������Χ

    while (1)
    {
        STMFLASH_Read(secpos * STM_SECTOR_SIZE + FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE / 2); //������������������
        for (i = 0; i < secremain; i++)                                                          //У������
        {
            if (STMFLASH_BUF[secoff + i] != 0XFFFF)
                break; //��Ҫ����
        }
        if (i < secremain) //��Ҫ����
        {
            //�����������
            /* Fill EraseInit structure*/
            EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
            EraseInitStruct.PageAddress = secpos * STM_SECTOR_SIZE + FLASH_BASE;
            EraseInitStruct.NbPages = 1;
            HAL_FLASHEx_Erase(&EraseInitStruct, &SECTORError);

            for (i = 0; i < secremain; i++) //����
            {
                STMFLASH_BUF[i + secoff] = pBuffer[i];
            }
            STMFLASH_Write_NoCheck(secpos * STM_SECTOR_SIZE + FLASH_BASE, STMFLASH_BUF, STM_SECTOR_SIZE / 2); //д����������
        }
        else
        {
            STMFLASH_Write_NoCheck(WriteAddr, pBuffer, secremain); //д�Ѿ������˵�,ֱ��д������ʣ������.
        }
        if (NumToWrite == secremain)
            break; //д�������
        else       //д��δ����
        {
            secpos++;                //������ַ��1
            secoff = 0;              //ƫ��λ��Ϊ0
            pBuffer += secremain;    //ָ��ƫ��
            WriteAddr += secremain;  //д��ַƫ��
            NumToWrite -= secremain; //�ֽ�(16λ)���ݼ�
            if (NumToWrite > (STM_SECTOR_SIZE / 2))
                secremain = STM_SECTOR_SIZE / 2; //��һ����������д����
            else
                secremain = NumToWrite; //��һ����������д����
        }
    };
    HAL_FLASH_Lock(); //����
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
