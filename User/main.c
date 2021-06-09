/**********************************************************************
��Ȩ���У�	���ش��¿Ƽ���2017.
��		����	http://www.miaowlabs.com
��		����	https://miaowlabs.taobao.com/
�� �� ��: 	main.c
��    ��:   ����ʵ����
��		��:   3.00
�������:   2017.03.01
��		Ҫ: 	

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

//�뼶����
void SecTask()
{
	if(SoftTimer[0])return;
	else{
		SoftTimer[0] = 1000;
	}
	g_RunTime++; 			// ��¼����ʱ��
	g_BatVolt = GetBatVoltage(); // ��ȡ��ص�ѹ
	
	if(StatusFlag)ResponseStatus();
	
	LEDToggle();
}


/*
	���������ţ����⣬���ƹ��ܺ�����stm32f10x_it.cִ���ļ��ĵδ�ʱ���жϷ�������ѭ��ִ�С�
*/
int main(void)
{	
	#if INFRARE_DEBUG_EN > 0
	char buff[32];	
	memset(buff, 0, sizeof(buff));
	#endif
	
	BspInit();				//��ʼ��BSP

	PIDInit(); 				//��ʼ��PID
	
	CarUpstandInit(); 	//��ʼ��ϵͳ����
	
	SysTick_Init();			//��ʼ����ʱ��	
	
	if(IsInfrareOK())
		g_iGravity_Offset = 1; //������⵽���Һ���ģ�飬�����ƫ��ֵ��
	
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
		
		SecTask();			//�뼶����

		if(SoftTimer[1] == 0)
		{// ÿ��20ms ִ��һ��
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
				if(IsUltraOK())UltraControl(0);	//����������ģʽ
	 		}
			if(g_CarRunningMode == ULTRA_AVOID_MODE)
			{
				UltraControl(1);	//����������ģʽ
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


