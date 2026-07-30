#include "ti_msp_dl_config.h"
#include "board.h"

volatile unsigned long tick_ms;
volatile uint32_t start_time;
uint8_t RxPacket[36];



////printf函数重定义
//int fputc(int ch, FILE *stream)
//{
//	//当串口0忙的时候等待，不忙的时候再发送传进来的字符
//	while( DL_UART_isBusy(UART_0_INST) == true );

//	DL_UART_Main_transmitDataBlocking(UART_0_INST, ch);

//	return ch;
//}

//int fputs(const char* restrict s,FILE* restrict stream)
//{
//   uint16_t i,len;
//   len = strlen(s);
//   for(i=0;i<len;i++)
//   {
//       DL_UART_Main_transmitDataBlocking(UART_0_INST,s[i]);
//   }
//   return len;
//}

//int puts(const char *_ptr)
//{
//    int count = fputs(_ptr,stdout);
//    count += fputs("\n",stdout);
//    return count;
//}


//返回SysTick计数值
uint32_t Systick_getTick(void)
{
	return (SysTick->VAL);
}


//ms阻塞延迟
void delay_ms(uint32_t ms)
{
	//超出能满足的最大延迟
	//if( ms > SysTickMAX_COUNT/(SysTickFre/1000) ) ms = SysTickMAX_COUNT/(SysTickFre/1000);
	for(int i=0;i<1000;i++)
	{
		delay_us(ms);
	}
}


void delay_us(uint32_t us)
{
	if( us > SysTickMAX_COUNT/(SysTickFre/1000000) ) us = SysTickMAX_COUNT/(SysTickFre/1000000);
	
	us = us*(SysTickFre/1000000); //单位转换
	
	//用于保存已走过的时间
	uint32_t runningtime = 0;
	
	//获得当前时刻的计数值
	uint32_t InserTick = Systick_getTick();
	
	//用于刷新实时时间
	uint32_t tick = 0;
	
	uint8_t countflag = 0;
	//等待延迟
	while(1)
	{
		tick = Systick_getTick();//刷新当前时刻计数值
		
		if( tick > InserTick ) countflag = 1;//出现溢出轮询,则切换走时的计算方式
		
		if( countflag ) runningtime = InserTick + SysTickMAX_COUNT - tick;
		else runningtime = InserTick - tick;
		
		if( runningtime>=us ) break;
	}

}


void delay_1us(unsigned long __us){ delay_us(__us); }
void delay_1ms(unsigned long ms){ delay_ms(ms); }
float Pitch, Roll, Yaw;
float GyroX, GyroY, GyroZ;
extern uint8_t sb;



//void UART_0_INST_IRQHandler(void)
//{
//    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST))
//    {
//        case DL_UART_MAIN_IIDX_DMA_DONE_RX:
//        {
//            /* 清除 DMA 完成中断标志 */
//            DL_UART_Main_clearInterruptStatus(UART_0_INST, 
//                DL_UART_MAIN_INTERRUPT_DMA_DONE_RX);
//            /* 一个完整的 35 字节包已存入 RxPacket，打印回显 */
//			           /* ---- 数据帧结构校验 ---- */
//            if (RxPacket[0]  == 0x59 &&
//                RxPacket[1]  == 0x53 &&
//                RxPacket[5]  == 0x20 &&
//                RxPacket[6]  == 0x0C &&
//                RxPacket[19]  == 0x40 &&
//                RxPacket[20]  == 0x0C 
//               )
//            {
//                /* 1. 提取角速度（小端，3 个 int32） */
//                int32_t raw_gx = (int32_t)(RxPacket[7]  | (RxPacket[8]  << 8) |
//                                           (RxPacket[9]  << 16) | (RxPacket[10] << 24));
//                int32_t raw_gy = (int32_t)(RxPacket[11] | (RxPacket[12] << 8) |
//                                           (RxPacket[13] << 16) | (RxPacket[14] << 24));
//                int32_t raw_gz = (int32_t)(RxPacket[15] | (RxPacket[16] << 8) |
//                                           (RxPacket[17] << 16) | (RxPacket[18] << 24));

//                GyroX = raw_gx * 0.000001f;
//                GyroY = raw_gy * 0.000001f;
//                GyroZ = raw_gz * 0.000001f;

//                /* 2. 提取欧拉角（小端，3 个 int32） */
//                int32_t raw_p = (int32_t)(RxPacket[21] | (RxPacket[22] << 8) |
//                                          (RxPacket[23] << 16) | (RxPacket[24] << 24));
//                int32_t raw_r = (int32_t)(RxPacket[25] | (RxPacket[26] << 8) |
//                                          (RxPacket[27] << 16) | (RxPacket[28] << 24));
//                int32_t raw_y = (int32_t)(RxPacket[29] | (RxPacket[30] << 8) |
//                                          (RxPacket[31] << 16) | (RxPacket[32] << 24));

//                Pitch = raw_p * 0.000001f;
//                Roll  = raw_r * 0.000001f;
//                Yaw   = raw_y * 0.000001f;

//                sb++;   // 有效帧计数
//            }
//			DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);
//            break;
//        }
//        case DL_UART_MAIN_IIDX_RX_TIMEOUT_ERROR:
//        {
//            /* 清除超时中断标志 */
//            DL_UART_Main_clearInterruptStatus(UART_0_INST, 
//                DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);

//            /* 超时说明数据流中断，DMA 未搬完 35 字节，需强行终止并清理 */
//            DL_DMA_disableChannel(DMA, DMA_CH0_CHAN_ID);

//            /* 清空残留 FIFO */
//            while (!DL_UART_isRXFIFOEmpty(UART_0_INST))
//            {
//                volatile uint8_t dummy = DL_UART_receiveData(UART_0_INST);
//                (void)dummy;
//            }

//            /* 重置 DMA 传输地址与尺寸（确保从新帧起始处接收） */
//            DL_DMA_setSrcAddr(DMA, DMA_CH0_CHAN_ID, 
//                              (uint32_t)(&UART_0_INST->RXDATA));
//            DL_DMA_setDestAddr(DMA, DMA_CH0_CHAN_ID, 
//                               (uint32_t)&RxPacket[0]);
//            DL_DMA_setTransferSize(DMA, DMA_CH0_CHAN_ID, 35);
//            DL_DMA_enableChannel(DMA, DMA_CH0_CHAN_ID);

//            break;
//        }
//        default:
//            break;
//    }
//}