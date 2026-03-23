//############################################################
// FILE:  ThreeHall.h
// Created on: 2017年1月5日
// Author: XQ
// summary: Header file  and definition
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP/STM32电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：314306105
//############################################################

#ifndef ThreeHall_H
#define ThreeHall_H

#include "main_user.h"
#include "IQ_math.h"

//HALL扇区默认固定电角度偏移
#define HALL_0_PHASE_SHIFT          (0UL)
#define HALL_60_PHASE_SHIFT       	(10923UL)
#define HALL_120_PHASE_SHIFT      	(21845UL)
#define HALL_180_PHASE_SHIFT      	(32768UL)
#define HALL_240_PHASE_SHIFT      	(43691UL)
#define HALL_300_PHASE_SHIFT      	(54613UL)

//用id慢速强拖得到的不同HALL状态切换下真实电角度
#define HALL_5_4_eleAngle       (40342UL)//(56829UL)
#define HALL_4_6_eleAngle      	(50369UL)  //65281
#define HALL_6_2_eleAngle      	(60915UL)
#define HALL_2_3_eleAngle      	(8600UL) //(23249UL)
#define HALL_3_1_eleAngle      	(18764UL) //(31121UL)
#define HALL_1_5_eleAngle      	(28314UL) //(42124UL)
//每个HALL扇区的真实电角度宽度
//#define HALL_5_eleAngleWitdh       (14705UL)
//#define HALL_4_eleAngleWitdh       (8760UL)
//#define HALL_6_eleAngleWitdh      	(9180UL)
//#define HALL_2_eleAngleWitdh      	(14069UL)
//#define HALL_3_eleAngleWitdh      	(7872UL)
//#define HALL_1_eleAngleWitdh      	(11003UL)				
/* 模差分（返回 0..65535） */
#define HALL_DIFF(prev, next)  ( (uint32_t)((uint32_t)(next) - (uint32_t)(prev)) & 0xFFFFU )
/* 每个扇区的宽度（按照环上顺序计算）：
   假设顺序为:
     HALL_3_1 -> HALL_1_5 -> HALL_5_4 -> HALL_4_6 -> HALL_6_2 -> HALL_2_3 -> (回到 HALL_3_1)
   则：
     HALL_1 在 HALL_3_1 与 HALL_1_5 之间
     HALL_5 在 HALL_1_5 与 HALL_5_4 之间
     HALL_4 在 HALL_5_4 与 HALL_4_6 之间
     HALL_6 在 HALL_4_6 与 HALL_6_2 之间
     HALL_2 在 HALL_6_2 与 HALL_2_3 之间
     HALL_3 在 HALL_2_3 与 HALL_3_1 之间
*/
#define HALL_1_eleAngleWitdh  HALL_DIFF(HALL_3_1_eleAngle, HALL_1_5_eleAngle)
#define HALL_5_eleAngleWitdh  HALL_DIFF(HALL_1_5_eleAngle, HALL_5_4_eleAngle)
#define HALL_4_eleAngleWitdh  HALL_DIFF(HALL_5_4_eleAngle, HALL_4_6_eleAngle)
#define HALL_6_eleAngleWitdh  HALL_DIFF(HALL_4_6_eleAngle, HALL_6_2_eleAngle)
#define HALL_2_eleAngleWitdh  HALL_DIFF(HALL_6_2_eleAngle, HALL_2_3_eleAngle)
#define HALL_3_eleAngleWitdh  HALL_DIFF(HALL_2_3_eleAngle, HALL_3_1_eleAngle)

					

#define HALL_Motor_FORWARD          (+1L)                   //正转 （逆时针）
#define HALL_Motor_STOP             (0L)                    //停转
#define HALL_Motor_REVERSAL          (-1L)                   //反转 顺时针）
///<120°电角度放置时的霍尔真值
#define HALL_5                  	(5UL)
#define HALL_1                  	(1UL)
#define HALL_3                  	(3UL)
#define HALL_2                  	(2UL)
#define HALL_6                  	(6UL)
#define HALL_4                  	(4UL)

#define WBuffer_MAX_SIZE     (6UL)   //通过过去n个扇区数量的数据来计算平均速度

typedef struct {
	    uint8_t       HallUVW[3];   // 读取三个霍尔的对应状态
	    uint8_t       Hall_State;   // 当前霍尔状态
	    uint8_t       OldHall_State; // 历史霍尔状态
	    uint8_t       HallLX_State;  // 当前和历史霍尔状态联接一个字节数据
		uint16_t      HALL_eleAngleWitdh[7];  //每个HALL状态扇区电角度宽度
	    uint8_t       HALLState_Filter_Count;  //HALL状态扇区判断滤波值
	
		uint8_t       Poles;             //电机极对数
	    int8_t        Move_State;        //电机旋转状态
		int8_t        Move_OldState;        //电机上一次旋转状态
        uint32_t      Initial_eleangle;     //电机初始电角度
	    uint32_t      Locked_Time;     //堵转时间，单位us
	    int32_t       Calibra_eleangle;       //转子校准电角度  （上一次扇区切换校准点电角度）
	    uint16_t      Calibra_Compensa_eleangle;       //转子校准电角度补偿值
	    int32_t       Predict_eleangle;           //霍尔转子预测电角度
		uint32_t      alpha_q15;
		int32_t       Predict_Oldeleangle;       //霍尔转子上一次预测电角度
	    int16_t       Predict_Compensa_eleangle;       //转子预测电角度补偿值

	    uint8_t       Speed_Start_Flag;  //初始速度计算开始标志位，1表示开始，0表示结束
		uint8_t       Speed_count_buffer_index;  //速度计数缓冲区索引
	    uint32_t      Speed_count_buffer[WBuffer_MAX_SIZE];      //过去WBuffer_MAX_SIZE个扇区中转子每转过一个扇区进入PWM中断次数
		uint32_t      Speed_count; 
		uint32_t      Speed_Oldcount; 
	    uint32_t      Speed_count_num;      //速度计数值总数，是上面数组元素值之和
	    uint32_t      Speed_Average_count;   //平均每个扇区进入PWM中断次数
	    uint32_t      Speed_Average_Oldcount;   //上一个平均每个扇区进入PWM中断次数
		uint32_t      Speed_Average_eleangle;   //平均一个FOC中断时间会多走多少电角度
	    int32_t       Speed_Increase_eleangle;     // = - 10923/(Speed_Average_count - Speed_Average_Oldcount)，
		                                           //预测下一个FOC中断时间会多走多少电角度，这里起到类似加速度的作用
		int32_t       Speed_Predict_Add_eleangle;     //预测一个FOC中断时间会走过多少电角度
		int32_t       Speed_Predict_OldAdd_eleangle;  //上一次预测一个FOC中断时间会走过多少电角度
	
	    uint16_t      Speed_RPM;         //电机旋转速度 
	    uint16_t      Speed_Calculate_Times;  //每进入多少次PWM中断计算一次速度
		uint16_t      Speed_PWM_Time;         //每过多少us进入一次PWM中断
		uint16_t      Speed_discretize_Time;  //离散化速度计算时间，单位us，等于Speed_Calculate_Times*Speed_PWM_Time

        /** 锁相环滤波 */
		float			pll_kp;
		float			pll_ki;
		float           pll_theta;               //<锁相环法估算的角度
		float			pll_omega;               //<锁相环法估算的速度

		IQSin_Cos       HallAngleSin_Cos;   //对霍尔电角度的正弦余弦值进行计算
		IQSin_Cos       PLLAngleSin_Cos;    //对锁相环电角度的正弦余弦值进行计算
		int32_t         PLL_Theta;          //PLL滤波后的电角度
		int32_t         PLL_DeltTheta;      //PLL电角度和霍尔电角度的error差值
		int32_t         PLL_DeltTheta_Sum;   //PLL电角度和霍尔电角度的error差值的积分
		int32_t         PLL_Omega;           //PLL预测的角速度
		int32_t         PLL_Omega_filter;      //PLL预测的角速度滤波，因为一般PLL预测的角速度比较噪声

} Hall;
extern Hall Hall_Three;
extern uint16_t   HallK1_RPM; 		// 一阶低通滤波系数 K1: 0.016/(T+0.016);  放大了1024倍，从浮点数转为了Q10格式定点数，截止频率10hz
extern uint16_t   HallK2_RPM;  	// 一阶低通滤波系数 K2: T/(T+0.016);
extern uint16_t   HallK1_Count; 
extern uint16_t   HallK2_Count; 
extern uint16_t   init_angle;		 

#define  Hall_DEFAULTS {0,0,0,0,0,  0,0,0,0,0,0,0,0,0,  0,0,0,0,0,0,0,0,0,0,0,  0,0,0,0  ,0,0,0,0} // 初始化参数

void ThreeHallPara_init(void);  //三霍尔计算角度参数初始化
static void ThreeHallangle_start(void);
uint8_t GET_HALLState_debounce_dynamic(void);
int32_t Hall_PLL_filter(Hall *Hall_Three);

int32_t hall_pll_filter(Hall *Hall_Three);
int32_t hall_pll_filter_float(Hall *Hall_Three);
//__attribute__((section("InRamCode"))) __attribute__((noinline)) 
uint32_t ThreeHallanglecale(void);  //三霍尔计算角度函数
uint16_t Hall_Three_Speedcale(void); //三霍尔角度计算速度函数
#endif /* ThreeHall_H_*/
//===========================================================================
// End of file.
//===========================================================================
