/**********************************************************************
��Ȩ���У�	���ش��¿Ƽ���2017.
��		����	http://www.miaowlabs.com
��		����	https://miaowlabs.taobao.com/
�� �� ��: 	manage.c
��    ��:   ����ʵ����
��		��:   3.00
�������:   2017.03.01
��		Ҫ: 	


***********************************************************************/

#include "manage.h"



const char FirmwareVer[] = "3.33";
const char EEPROMVer[]  = "2.00";
const char MCUVer[] = "STM32F103C8T6";

//ϵͳ����ʱ���������������ʼ������ÿ����1
unsigned short  g_RunTime = 0;

//��ص�ѹ��ʵ��ֵ*100
unsigned short  g_BatVolt=0;

//С������ģʽ:ң��ģʽ������Ѱ��ģʽ�����������ϡ�����������ģʽ
unsigned char g_CarRunningMode = ULTRA_AVOID_MODE;//Ĭ��Ϊң��ģʽ

//���¶�����manage.h��
//#define CONTROL_MODE 			1   ң��ģʽ
//#define INFRARED_TRACE_MODE 	2    ����Ѱ��ģʽ
//#define ULTRA_FOLLOW_MODE		3       ����������ģʽ
//#define ULTRA_AVOID_MODE	   4      ����������ģʽ
