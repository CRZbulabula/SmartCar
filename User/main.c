/**********************************************************************
版权所有：	喵呜创新科技，2017.
官		网：	http://www.miaowlabs.com
淘		宝：	https://miaowlabs.taobao.com/
文 件 名: 	main.c
作    者:   喵呜实验室
版		本:   3.00
完成日期:   2017.03.01
概		要: 	

***********************************************************************/
#include "stm32f10x.h"
#include "stdio.h"
#include "usart.h"
#include "SysTick.h"
#include "control.h"
#include "debug.h"
#include "communicate.h"
#include "dataflash.h"
#include "common.h"
#include "motor.h"
#include "display.h"
#include "bsp.h"
#include "ADC.h"
#include "ultrasonic.h"
#include "infrare.h"
#include "manage.h"

//秒级任务
void SecTask()
{
	if(SoftTimer[0])return;
	else{
		SoftTimer[0] = 1000;
	}
	g_RunTime++; 			// 记录运行时间
	g_BatVolt = GetBatVoltage(); // 读取电池电压
	
	if(StatusFlag)ResponseStatus();
	
	LEDToggle();
}


/*
	主函数入门，另外，控制功能函数在stm32f10x_it.c执行文件的滴答定时器中断服务函数里循环执行。
*/
int main(void)
{	
	#if INFRARE_DEBUG_EN > 0
	char buff[32];	
	memset(buff, 0, sizeof(buff));
	#endif
	
	BspInit();				//初始化BSP

	PIDInit(); 				//初始化PID
	
	CarUpstandInit(); 	//初始化系统参数
	
	SysTick_Init();			//初始化定时器	
	
	if(IsInfrareOK())
		g_iGravity_Offset = 1; //若果检测到悬挂红外模块，则更改偏移值。
	
	ShowHomePageInit();
	
	//SoftTimer[3] = 2000;
	//while(SoftTimer[3]) {}
 
	//Steer(0, 3);
	SPEED_FORCE_EQUAL = 0;
	while (1)
	{
		// Task1(SoftTimer);
		
		if(g_CarRunningMode == INFRARED_TRACE_MODE && SoftTimer[2] == 0)
		{
			SoftTimer[2] = 10;
			TailingControl();
		}
		
		SecTask();			//秒级任务

		if(SoftTimer[1] == 0)
		{// 每隔20ms 执行一次
			SoftTimer[1] = 20;
			ResponseIMU();			
			DebugService();			
			//Parse(Uart3Buffer);
		}
   	
		if(SoftTimer[3] == 0)
		{
			SoftTimer[3] = 100;	// todo: to be modified
			
			// ShowHomePage();
			Read_Distane();

			if(g_CarRunningMode == ULTRA_FOLLOW_MODE){
				if(IsUltraOK())UltraControl(0);	//超声波跟随模式
	 		}
			if(g_CarRunningMode == ULTRA_AVOID_MODE)
			{
				UltraControl(1);	//超声波避障模式
				if (!IsUltraOK() && g_iStateReadyChange) {
					g_CarRunningMode = 5;
				}
			}
		}
		
		if (g_CarRunningMode == 5 && SoftTimer[2] == 0) {
			SoftTimer[2] = 1000;
			sprintf(buff, "Stop!\r\n");
			ShowStr(buff);
		}
		

		/*if (SoftTimer[3] == 0) {
			SoftTimer[3] = 500;
			sprintf(buff, "order: %d\n", g_iOrderPosition);
			//sprintf(buff, "direction: %d, position: %d cnt: %d %d\n\0", 
			//g_iDestinationRelatedDirection, g_iWallRelatedPosition, g_iLeftTurnRoundCnt, g_iRightTurnRoundCnt);
			ShowStr(buff);
		}*/
	}
}


/******************* (C) COPYRIGHT 2016 MiaowLabs Team *****END OF FILE************/


