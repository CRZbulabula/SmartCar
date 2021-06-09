/**********************************************************************
版权所有：	  喵呜实验室MiaowLabs，2017.
官		网：	http://www.miaowlabs.com
淘		宝：	https://miaowlabs.taobao.com/
文 件 名: 	  control.c
作    者:   喵呜实验室MiaowLabs
版		本:   3.00
完成日期:   2017.03.01
概		要: 	


***********************************************************************/
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "control.h"
#include "debug.H"
#include "MPU6050.H"
#include "communicate.h"
#include "bsp.h"
#include "ultrasonic.h"
#include "infrare.h"
#include "display.h"
#include "manage.h"

unsigned char g_u8MainEventCount;
unsigned char g_u8SpeedControlCount;
unsigned char g_u8SpeedControlPeriod;
unsigned char g_u8DirectionControlPeriod;
unsigned char g_u8DirectionControlCount;

unsigned char g_cMotorDisable = 0;//值等于0时电机正常转动，否则停止转动


int g_iGravity_Offset = 0;

/******电机控制参数******/
float g_fSpeedControlOut;
float g_fSpeedControlOutOld;
float g_fSpeedControlOutNew;
float g_fAngleControlOut;
float g_fLeftMotorOut;
float g_fRightMotorOut;

/* 任务1专用 */
float TASK1_LEFT_OFFSET, TASK1_RIGHT_OFFSET;

/******速度控制参数******/

short  g_s16LeftMotorPulse;
short  g_s16RightMotorPulse;

int  g_s32LeftMotorPulseOld;
int  g_s32RightMotorPulseOld;
int  g_s32LeftMotorPulseSigma;
int  g_s32RightMotorPulseSigma;
int  g_s32MotorPulseDelta;

float g_fCarSpeed;
float g_iCarSpeedSet;
float g_fCarSpeedOld;
float g_fCarPosition;

/*-----角度环和速度环PID控制参数-----*/
PID_t g_tCarAnglePID={17.0, 0, 23.0};	//*5 /10
PID_t g_tCarSpeedPID={15.25, 1.08, 0};	//i/10
/******蓝牙控制参数******/
float g_fBluetoothSpeed;
float g_fBluetoothDirection;
float g_fBluetoothDirectionOld;
float g_fBluetoothDirectionNew;
float g_fBluetoothDirectionOut;

float g_fCarAngle;         	//
float g_fGyroAngleSpeed;		//     			
float g_fGravityAngle;			//

// 直行计数
int g_iMoveCnt = 0;

int SPEED_FORCE_EQUAL = 1;

// 转弯计数
int g_iLeftTurnRoundCnt = 0;
int g_iRightTurnRoundCnt = 0;

int g_iTurnFlag = 0, g_iTurnFinished = 0;
int g_iOrderPosition = 1;
int g_iTurnOrder[4] = {1, 1, -1, -1};

int g_iDestinationRelatedDirection = 0;	// 相对于终点的方向：-1表示当前正在向左走  0表示当前正在直行    1表示当前正在向右走
int g_iWallRelatedPosition = 0;			// 相对于墙的位置：  -1表示在左边墙向前走  1表示在右边墙向前走  0表示其他情况

int g_iStateReadyChange = 0;

static int AbnormalSpinFlag = 0;
/***************************************************************
** 函数名称: CarUpstandInit
** 功能描述: 全局变量初始化函数
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室MiaowLabs
** 淘  宝：  https://miaowlabs.taobao.com/
** 日　期:   2014年08月01日
***************************************************************/
void CarUpstandInit(void)
{
	//g_iAccelInputVoltage_X_Axis = g_iGyroInputVoltage_Y_Axis = 0;
	g_s16LeftMotorPulse = g_s16RightMotorPulse = 0;
	g_s32LeftMotorPulseOld = g_s32RightMotorPulseOld = 0;
	g_s32LeftMotorPulseSigma = g_s32RightMotorPulseSigma = 0;

	g_fCarSpeed = g_fCarSpeedOld = 0;
	g_fCarPosition = 0;
	g_fCarAngle    = 0;
	g_fGyroAngleSpeed = 0;
	g_fGravityAngle   = 0;

	g_fAngleControlOut = g_fSpeedControlOut = g_fBluetoothDirectionOut = 0;
	g_fLeftMotorOut    = g_fRightMotorOut   = 0;
	g_fBluetoothSpeed  = g_fBluetoothDirection = 0;
	g_fBluetoothDirectionNew = g_fBluetoothDirectionOld = 0;

  g_u8MainEventCount=0;
	g_u8SpeedControlCount=0;
 	g_u8SpeedControlPeriod=0;
}


/***************************************************************
** 函数名称: AbnormalSpinDetect
** 功能描述: 电机转速异常检测      
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室MiaowLabs
** 日　期:   2017年4月26日
***************************************************************/

void AbnormalSpinDetect(short leftSpeed,short rightSpeed)
{
	static unsigned short count = 0;
	
	//速度设置为0时检测，否则不检测
	if(g_iCarSpeedSet==0)
	{
		if(((leftSpeed>30)&&(rightSpeed>30)&&(g_fCarAngle > -30) && (g_fCarAngle < 30))
			||((leftSpeed<-30)&&(rightSpeed<-30))&&(g_fCarAngle > -30) && (g_fCarAngle < 30))
		{// 左右电机转速大于30、方向相同、持续时间超过250ms，且车身角度不超过30度，则判断为悬空空转
			count++;
			if(count>50){
				count = 0;
				AbnormalSpinFlag = 1;
			}
		}
		else{
			count = 0;
		}
	}
	else{
		count = 0;
	}
}

/***************************************************************
** 函数名称: LandingDetect
** 功能描述: 小车着地检测      
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室MiaowLabs
** 日　期:   2017年4月26日
***************************************************************/
void LandingDetect(void)
{
	static float lastCarAngle = 0;
	static unsigned short count = 0,count1 = 0;
	
	if(AbnormalSpinFlag == 0)return;
	
	// 小车角度5°~-5°启动检测
	if((g_fCarAngle > -5) && (g_fCarAngle < 5))
	{
		count1++;
		if(count1 >= 50)
		{//每隔250ms判断一次小车角度变化量，变化量小于0.8°或大于-0.8°判断为小车静止
			count1 = 0;
			if(((g_fCarAngle - lastCarAngle) < 0.8) && ((g_fCarAngle - lastCarAngle) > -0.8))
			{
				count++;
				if(count >= 4){
					count = 0;
					count1 = 0;
					g_fCarPosition = 0;
					AbnormalSpinFlag = 0;
				}
			}
			else{
				count = 0;
			}
			lastCarAngle = g_fCarAngle;
		}
	}
	else
	{
		count1 = 0;
		count = 0;
	}
}

/***************************************************************
** 函数名称: MotorManage
** 功能描述: 电机使能/失能控制      
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室MiaowLabs
** 日　期:   2017年4月26日
***************************************************************/
void MotorManage(void)
{

	AbnormalSpinDetect(g_s16LeftMotorPulse, g_s16RightMotorPulse);
		
	LandingDetect();
	
	if(AbnormalSpinFlag)
	{	
		g_cMotorDisable |= (0x01<<1);
	}
	else
	{
		g_cMotorDisable &= ~(0x01<<1);
	}
	
	if(g_fCarAngle > 30 || g_fCarAngle < (-30))
	{
		g_cMotorDisable |= (0x01<<2);
	}
	else
	{
		g_cMotorDisable &= ~(0x01<<2);
	}
	
}

/***************************************************************
** 函数名称: SetMotorVoltageAndDirection
** 功能描述: 电机转速及方向控制函数             
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室MiaowLabs
** 淘  宝：  https://miaowlabs.taobao.com/
** 日　期:   2018年08月27日
***************************************************************/
void SetMotorVoltageAndDirection(int i16LeftVoltage,int i16RightVoltage)
{
	  if(i16LeftVoltage<0)
    {	
			GPIO_SetBits(GPIOA, GPIO_Pin_3 );				    
      GPIO_ResetBits(GPIOA, GPIO_Pin_4 );
      i16LeftVoltage = (-i16LeftVoltage);
    }
    else 
    {	
      GPIO_SetBits(GPIOA, GPIO_Pin_4 );				    
      GPIO_ResetBits(GPIOA, GPIO_Pin_3 ); 
    }

    if(i16RightVoltage<0)
    {	
     	GPIO_SetBits(GPIOB, GPIO_Pin_0 );				    
      GPIO_ResetBits(GPIOB, GPIO_Pin_1 );
      i16RightVoltage = (-i16RightVoltage);
    }
    else
    {
			GPIO_SetBits(GPIOB, GPIO_Pin_1 );				    
			GPIO_ResetBits(GPIOB, GPIO_Pin_0 );	      
    }

	if(i16RightVoltage > MOTOR_OUT_MAX)  
	{
		i16RightVoltage = MOTOR_OUT_MAX;
	}
	if(i16LeftVoltage > MOTOR_OUT_MAX)
	{
	   i16LeftVoltage = MOTOR_OUT_MAX;
	}  
	
	if(g_cMotorDisable)
	{
		TIM_SetCompare1(TIM3,0);
		TIM_SetCompare2(TIM3,0); 
	}
	else
	{
		TIM_SetCompare1(TIM3,i16RightVoltage);
		TIM_SetCompare2(TIM3,i16LeftVoltage);
	}
}


/***************************************************************
** 函数名称: MotorOutput
** 功能描述: 电机输出函数
             将直立控制、速度控制、方向控制的输出量进行叠加,并加
			 入死区常量，对输出饱和作出处理。
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室MiaowLabs
** 淘  宝：  https://miaowlabs.taobao.com/ 
** 日　期:   2014年08月01日
***************************************************************/
void MotorOutput(void)
{
	g_fLeftMotorOut  = g_fAngleControlOut - g_fSpeedControlOut - g_fBluetoothDirection ;	//这里的电机输出等于角度环控制量 + 速度环外环,这里的 - g_fSpeedControlOut 是因为速度环的极性跟角度环不一样，角度环是负反馈，速度环是正反馈
	g_fRightMotorOut = g_fAngleControlOut - g_fSpeedControlOut + g_fBluetoothDirection ;

	g_fLeftMotorOut -= TASK1_LEFT_OFFSET;
	g_fRightMotorOut -= TASK1_RIGHT_OFFSET;

	if (SPEED_FORCE_EQUAL) {
		if (g_s32MotorPulseDelta > 0) {
			g_fRightMotorOut += 0.25 * g_s32MotorPulseDelta;
		} else {
			g_fLeftMotorOut += 0.25 * g_s32MotorPulseDelta;
		}
	}

	/*增加死区常数*/
	if((int)g_fLeftMotorOut>0)       g_fLeftMotorOut  += MOTOR_OUT_DEAD_VAL;
	else if((int)g_fLeftMotorOut<0)  g_fLeftMotorOut  -= MOTOR_OUT_DEAD_VAL;
	if((int)g_fRightMotorOut>0)      g_fRightMotorOut += MOTOR_OUT_DEAD_VAL;
	else if((int)g_fRightMotorOut<0) g_fRightMotorOut -= MOTOR_OUT_DEAD_VAL;

	/*输出饱和处理，防止超出PWM范围*/			
	if((int)g_fLeftMotorOut  > MOTOR_OUT_MAX)	g_fLeftMotorOut  = MOTOR_OUT_MAX;
	if((int)g_fLeftMotorOut  < MOTOR_OUT_MIN)	g_fLeftMotorOut  = MOTOR_OUT_MIN;
	if((int)g_fRightMotorOut > MOTOR_OUT_MAX)	g_fRightMotorOut = MOTOR_OUT_MAX;
	if((int)g_fRightMotorOut < MOTOR_OUT_MIN)	g_fRightMotorOut = MOTOR_OUT_MIN;
	
    SetMotorVoltageAndDirection((int)g_fLeftMotorOut,(int)g_fRightMotorOut);
}



void GetMotorPulse(void)  //采集电机速度脉冲
{ 	
  g_s16LeftMotorPulse = TIM_GetCounter(TIM2);     
  g_s16RightMotorPulse= -TIM_GetCounter(TIM4);
  TIM2->CNT = 0;
  TIM4->CNT = 0;   //清零

  g_s32LeftMotorPulseSigma +=  g_s16LeftMotorPulse;
  g_s32RightMotorPulseSigma += g_s16RightMotorPulse; 
  g_s32MotorPulseDelta = g_s16LeftMotorPulse - g_s16RightMotorPulse;
	
	g_iMoveCnt -= (g_s16LeftMotorPulse + g_s16RightMotorPulse) * 0.5;
	g_iLeftTurnRoundCnt -= g_s16LeftMotorPulse;
	g_iRightTurnRoundCnt -= g_s16RightMotorPulse;

	if (g_iTurnFlag == 1 && g_iLeftTurnRoundCnt < 0 && g_iRightTurnRoundCnt > 0) {
		// 右转结束
		g_iTurnFlag = 0;
		g_iTurnFinished = 1;
	}

	if (g_iTurnFlag == -1 && g_iLeftTurnRoundCnt > 0 && g_iRightTurnRoundCnt < 0) {
		// 左转结束
		g_iTurnFlag = 0;
		g_iTurnFinished = 1;
	}
}

/***************************************************************
** 作　  者: MiaowLabs Team
** 官    网：http://www.miaowlabs.com
** 淘    宝：https://miaowlabs.taobao.com/
** 日　  期: 2015年11月29日
** 函数名称: AngleCalculate
** 功能描述: 角度环计算函数           
** 输　  入:   
** 输　  出:   
** 备    注: 
********************喵呜实验室MiaowLabs版权所有**************************
***************************************************************/
void AngleCalculate(void)
{
	//-------加速度--------------------------
	//量程为±2g时，灵敏度：16384 LSB/g
    g_fGravityAngle = atan2(g_fAccel_y/16384.0,g_fAccel_z/16384.0) * 180.0 / 3.14;
	  g_fGravityAngle = g_fGravityAngle - g_iGravity_Offset;

	//-------角速度-------------------------
	//范围为2000deg/s时，换算关系：16.4 LSB/(deg/s)
	g_fGyro_x  = g_fGyro_x / 16.4;  //计算角速度值			   
	g_fGyroAngleSpeed = g_fGyro_x;	
	
	//-------互补滤波---------------
	g_fCarAngle = 0.98 * (g_fCarAngle + g_fGyroAngleSpeed * 0.005) + 0.02 *	g_fGravityAngle;
}
/***************************************************************
** 作　  者: 喵呜实验室MiaowLabs
** 官    网：http://www.miaowlabs.com
** 淘    宝：https://miaowlabs.taobao.com/
** 日　  期: 2018年08月27日
** 函数名称: AngleControl
** 功能描述: 角度环控制函数           
** 输　  入:   
** 输　  出:   
** 备    注: 
********************喵呜实验室MiaowLabs版权所有**************************
***************************************************************/
void AngleControl(void)	 
{
	g_fAngleControlOut =  (CAR_ANGLE_SET-g_fCarAngle) * g_tCarAnglePID.P *5 + \
	(CAR_ANGLE_SPEED_SET-g_fGyroAngleSpeed) * (g_tCarAnglePID.D /10);
}



/***************************************************************
** 函数名称: SpeedControl
** 功能描述: 速度环控制函数
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室MiaowLabs
** 淘  宝：  https://miaowlabs.taobao.com/
** 日　期:   2014年08月01日
***************************************************************/

void SpeedControl(void)
{
  	float fP,fI;   	
	float fDelta;
	
	
	g_fCarSpeed = (g_s32LeftMotorPulseSigma  + g_s32RightMotorPulseSigma ) * 0.5 ;
  g_s32LeftMotorPulseSigma = g_s32RightMotorPulseSigma = 0;	  //全局变量 注意及时清零
    	
	g_fCarSpeed = 0.7 * g_fCarSpeedOld + 0.3 * g_fCarSpeed ;//低通滤波，使速度更平滑
	g_fCarSpeedOld = g_fCarSpeed;

	fDelta = CAR_SPEED_SET;
	fDelta -= g_fCarSpeed;   
	
	fP = fDelta * (g_tCarSpeedPID.P);
  fI = fDelta * (g_tCarSpeedPID.I/10.0);

	g_fCarPosition += fI;
	g_fCarPosition += g_fBluetoothSpeed;	  
	
//积分上限设限
	if((s16)g_fCarPosition > CAR_POSITION_MAX)    g_fCarPosition = CAR_POSITION_MAX;
	if((s16)g_fCarPosition < CAR_POSITION_MIN)    g_fCarPosition = CAR_POSITION_MIN;
	
	g_fSpeedControlOutOld = g_fSpeedControlOutNew;
  g_fSpeedControlOutNew = fP + g_fCarPosition;
}
/***************************************************************
** 函数名称: SpeedControlOutput
** 功能描述: 速度环控制输出函数-分多步逐次逼近最终输出，尽可能将对直立环的干扰降低。
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室MiaowLabs
** 淘  宝：  https://miaowlabs.taobao.com/
** 日　期:   2014年08月01日
***************************************************************/
void SpeedControlOutput(void)
{
  float fValue;
  fValue = g_fSpeedControlOutNew - g_fSpeedControlOutOld ;
  g_fSpeedControlOut = fValue * (g_u8SpeedControlPeriod + 1) / SPEED_CONTROL_PERIOD + g_fSpeedControlOutOld; 
}


/***************************************************************
** 函数名称: Scale
** 功能描述: 量程归一化处理
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室MiaowLabs
** 淘  宝：  https://miaowlabs.taobao.com/
** 日　期:   2014年08月01日
***************************************************************/
float Scale(float input, float inputMin, float inputMax, float outputMin, float outputMax) { 
  float output;
  if (inputMin < inputMax)
    output = (input - inputMin) / ((inputMax - inputMin) / (outputMax - outputMin));
  else
    output = (inputMin - input) / ((inputMin - inputMax) / (outputMax - outputMin));
  if (output > outputMax)
    output = outputMax;
  else if (output < outputMin)
    output = outputMin;
  return output;
}

/***************************************************************
** 函数名称: Steer
** 功能描述: 遥控速度及方向处理函数
** 输　入:   
** 输　出:   
** 全局变量: 
** 作　者:   喵呜实验室MiaowLabs
** 淘  宝：  https://miaowlabs.taobao.com/
** 日　期:   2014年08月01日
***************************************************************/
void Steer(float direct, float speed)
{
	if(direct > 0)
		g_fBluetoothDirection = Scale(direct, 0, 10, 0, 400);
	else
		g_fBluetoothDirection = -Scale(direct, 0, -10, 0, 400);

	if(speed > 0)
		g_iCarSpeedSet = Scale(speed, 0, 10, 0, 70);
	else
		g_iCarSpeedSet = -Scale(speed, 0, -10, 0, 70);

}

/***************************************************************
** 作　  者: Songyibiao
** 官    网：http://www.miaowlabs.com
** 淘    宝：https://miaowlabs.taobao.com/
** 日　  期: 20160415
** 函数名称: UltraControl
** 功能描述: 超声波跟随/避障           
** 输　  入:   
** 输　  出:   
** 备    注: 
********************喵呜实验室MiaowLabs版权所有**************************/
void UltraControl(int mode)
{
	if(mode == 0)
	{
		if((Distance >= 0) && (Distance<= 12))
		{//距离小于12cm则后退
			Steer(0, -4);
		}
		else if((Distance> 18) && (Distance<= 30))	
		{//距离大于18cm且小于30则前进
			Steer(0, 4);
		}
		else
			Steer(0, 0);
	}
	else if(mode == 1)
	{
		if((Distance >= 0) && (Distance<= 20))
		{
			SPEED_FORCE_EQUAL = 0;
			if (g_iTurnOrder[g_iOrderPosition] == 1) {
				// 开始右转
				Steer(5, 0);
				g_iLeftTurnRoundCnt = TURN_CNT;
				g_iRightTurnRoundCnt = -TURN_CNT;
				g_iTurnFlag = 1;
			} else {
				// 开始左转
				Steer(-5, 0);
				g_iLeftTurnRoundCnt = -TURN_CNT;
				g_iRightTurnRoundCnt = TURN_CNT;
				g_iTurnFlag = -1;
			}
		}
		
		if (g_iTurnFinished) {
			// 转弯完成
			SPEED_FORCE_EQUAL = 1;
			g_iStateReadyChange = 1;
			Steer(0, 3);
			g_iTurnFinished = 0;
			g_iOrderPosition = (g_iOrderPosition + 1) % 4;
		}
	}
}

/***************************************************************
** 作　  者: MiaowLabs Team
** 官    网：http://www.miaowlabs.com
** 淘    宝：https://miaowlabs.taobao.com/
** 日　  期: 20160415
** 函数名称: TailingControl
** 功能描述: 红外寻迹           
** 输　  入:   
** 输　  出:   
** 备    注: 
********************喵呜实验室MiaowLabs版权所有**************************
***************************************************************/
void TailingControl(void)
{
#if INFRARE_DEBUG_EN > 0
	char buff[32] = {0};
#endif
	char result;
	float direct = 0;
	float speed = 0;
	int la,ra,lb,rb;
	la = ra = lb = rb = 0;
	if (!IsInfrareOK()) {
		g_CarRunningMode = ULTRA_AVOID_MODE;
		Steer(0, 3);
		return;
	}
	result = InfraredDetect();
	
	if(result & infrared_channel_Lc){
	 	direct = -10;
		lb = 1;
	}
	else if(result & infrared_channel_La){
		direct = -4;
		la = 1;
	}
	if(result & infrared_channel_Ra){
		direct += 4;
		ra = 1;
	}
	if (result & infrared_channel_Rc){
	 	direct = 10;
		rb = 1;
	}

	if (direct == 0) speed = 2;
	else speed = 1;

	Steer(direct, speed);

#if INFRARE_DEBUG_EN > 0
	sprintf(buff, "lb:%d, la:%d, ra:%d, rb:%d\r\n",lb, la, ra, rb);
	ShowStr(buff);
	//DebugOutStr(buff)
#endif
}

/**
 * 任务1
 */
void Task1(unsigned short* SoftTimer)
{
	char buff[32];
	int WAIT = 3000;
	int MAXF = 400, MINF = 300;
	int MOVE_CNT = 6000, ROUND_CNT = 3000;
	memset(buff, 0, sizeof(buff));

	sprintf(buff, "wait\n\0");
	ShowStr(buff);
	SoftTimer[3] = WAIT;
	while (SoftTimer[3] > 0) {}
	sprintf(buff, "forward\n\0");
	ShowStr(buff);
	g_iMoveCnt = MOVE_CNT;
	Steer(0, 3);
	while (g_iMoveCnt > 0) {}
	Steer(0, 0);
	
	sprintf(buff, "wait\n\0");
	ShowStr(buff);
	SoftTimer[3] = WAIT;
	while (SoftTimer[3] > 0) {}
	sprintf(buff, "backward\n\0");
	ShowStr(buff);
	g_iMoveCnt = -MOVE_CNT;
	Steer(0, -3);
	while (g_iMoveCnt < 0) {}
	Steer(0, 0);

	sprintf(buff, "wait\n\0");
	ShowStr(buff);
	SPEED_FORCE_EQUAL = 0;
	SoftTimer[3] = 2000;
	while (SoftTimer[3] > 0) {}
	sprintf(buff, "left\n\0");
	ShowStr(buff);
	// 开始左转
	TASK1_LEFT_OFFSET = MINF;
	TASK1_RIGHT_OFFSET = MAXF;
	g_iRightTurnRoundCnt = ROUND_CNT;
	while (g_iRightTurnRoundCnt > 0) {}
	TASK1_LEFT_OFFSET = 0;
	TASK1_RIGHT_OFFSET = 0;
	Steer(0, 0);

	sprintf(buff, "wait\n\0");
	ShowStr(buff);
	SoftTimer[3] = 2000;
	while (SoftTimer[3] > 0) {}
	sprintf(buff, "right\n\0");
	ShowStr(buff);
	// 开始右转
	TASK1_LEFT_OFFSET = MAXF;
	TASK1_RIGHT_OFFSET = MINF;
	g_iLeftTurnRoundCnt = ROUND_CNT;
	while (g_iLeftTurnRoundCnt > 0) {}
	TASK1_LEFT_OFFSET = 0;
	TASK1_RIGHT_OFFSET = 0;
	Steer(0, 0);
}
