/**********************************************************************
��Ȩ���У�	���ش��¿Ƽ���2017.
��		����	http://www.miaowlabs.com
��		����	https://miaowlabs.taobao.com/
�� �� ��: 	systick.c
��    ��:   ����ʵ����
��		��:   3.00
�������:   2017.03.01
��		Ҫ: 	


***********************************************************************/
#include "stm32f10x.h"

#include "SysTick.h"

// ϵͳ������ʱ�����ֱ���Ϊ1ms���ݼ�����
unsigned short SoftTimer[5] = {0, 0, 0, 0, 0};

void SoftTimerCountDown(void)
{
	char i;
	for(i = 0;  i < 5; i++){
		if(SoftTimer[i] > 0)SoftTimer[i]--;
	}
}

/*
 * ��������SysTick_Init
 * ����  ������ϵͳ�δ�ʱ�� SysTick
 * ����  ����
 * ���  ����
 * ����  ���ⲿ���� 
 */
void SysTick_Init(void)
{
	/* SystemFrequency / 100     10ms�ж�һ��
	   SystemFrequency / 1000    1ms�ж�һ��
	 * SystemFrequency / 100000	 10us�ж�һ��
	 * SystemFrequency / 1000000 1us�ж�һ��
	 */
	SystemCoreClockUpdate();

	if (SysTick_Config(SystemCoreClock / 1000))	// ST3.5.0��汾
	{ 
		/* Capture error */ 
		while (1);
	}
	// �����δ�ʱ��  
	SysTick->CTRL |=  SysTick_CTRL_ENABLE_Msk;
}


