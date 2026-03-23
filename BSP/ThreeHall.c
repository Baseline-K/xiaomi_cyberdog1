//############################################################
// FILE: ThreeHall.c
// Created on: 2017年1月18日
// Author: XQ
// summary: ThreeHall
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//版权所有，盗版必究
//DSP/STM32电机控制开发板
//硕历电子
//网址: https://shuolidianzi.taobao.com
//Author-QQ: 616264123
//电机控制QQ群：314306105
//############################################################
#include "ThreeHall.h"
#include "tim.h"

#include <string.h>
#include "FOC_run.h"

Hall Hall_Three = Hall_DEFAULTS;
/*下面的一阶低通滤波用于转子扇区变化和1ms事件中，假设转子速度为3000转一圈12个扇区，则一个扇区变化时间为1.67ms，
  我们大概可以知道下面的滤波采样时间大于1ms-2ms；
	关于截止频率的确定：Hall_Three.Speed_count和转速强相关，Hall_Three.step_angle也和转速相关性较强，
	在实际运行中，电机的速度不会剧烈变化，转速的变化频率通常会低于电机的旋转频率。对于大部分电机控制应用，转速的变化频率
	通常在电机的旋转频率的十分之一到二十分之一。这里假设电机一秒转60圈60HZ，那么截止频率选择10HZ是很合理的；
	不过这里有一个需要注意的地方，低速时这两个变量的采样时间T比较大，则K1不变时等效为RC变大，表现为低速是截止频率
	其实会远小于10HZ，系统非常不灵敏，这里我们对采样时间变化的前两个变量的滤波使用模糊控制实时改变滤波系数
	
	Hall_Three.Speed_ele_angleIQ为1ms时间差时电机电角度的变化，同样和转速强相关，因此截止频率和前面两个变量一致是没问题的
*/
uint16_t   HallK1_RPM = 800; 		// 一阶低通滤波系数 K1: 0.016/(T+0.016);  放大了1024倍，从浮点数转为了Q10格式定点数，截止频率10hz
uint16_t   HallK2_RPM = 224;  	// 一阶低通滤波系数 K2: T/(T+0.016);
  
/*初始的滤波系数，假设60rpm，则采样时间T = 1/(转速/60*12)大概为100ms，此时计算如下*/
uint16_t   HallK1_Count = 141; 		// 一阶低通滤波系数 K1: 0.016/(T+0.016);  放大了1024倍，从浮点数转为了Q10格式定点数，截止频率10hz
uint16_t   HallK2_Count = 883;  	// 一阶低通滤波系数 K2: T/(T+0.016);

#define HallK1_Count_RC     16//0.016*1024   Q16.10格式

uint16_t   init_angle = 30000;			//2对级   霍尔的初始角度37500  0---65536  0---360度，三个霍尔间隔120度电角度放置
void  ThreeHallPara_init(void)
{
	memset(&Hall_Three, 0, sizeof(Hall_Three)); 

	Hall_Three.Speed_Start_Flag = 1;
	Hall_Three.Speed_count_buffer_index = 0;
    for(int i=0;i < WBuffer_MAX_SIZE;i++)
    {
	   Hall_Three.Speed_count_buffer[i] = 0;   //初始化每个扇区的计数值
    }
    //初始化每个扇区电角度宽度
    Hall_Three.HALL_eleAngleWitdh[1] = HALL_1_eleAngleWitdh;
	Hall_Three.HALL_eleAngleWitdh[2] = HALL_2_eleAngleWitdh;
	Hall_Three.HALL_eleAngleWitdh[3] = HALL_3_eleAngleWitdh;
	Hall_Three.HALL_eleAngleWitdh[4] = HALL_4_eleAngleWitdh;
	Hall_Three.HALL_eleAngleWitdh[5] = HALL_5_eleAngleWitdh;
	Hall_Three.HALL_eleAngleWitdh[6] = HALL_6_eleAngleWitdh;
   	
	Hall_Three.Poles=2;   
	Hall_Three.Initial_eleangle = init_angle;  // 电机初始电角度
	Hall_Three.HALLState_Filter_Count = 20;  //HALL电平变化连续n次一样才判断为正确HALL电平

	Hall_Three.Speed_Calculate_Times = 10;  //每进入10次PWM中断计算一次速度
	Hall_Three.Speed_PWM_Time = 1000000/FOC_Frequency;       //每过多少us进入一次PWM中断
	Hall_Three.Speed_discretize_Time = Hall_Three.Speed_Calculate_Times*Hall_Three.Speed_PWM_Time;
	Hall_Three.Locked_Time = 1000000;  //堵转时间初始为1s

	ThreeHallangle_start();  //给定HALL初始预测电角度
}

//刚上电时不知道位置，利用霍尔初始状态给一个大概电角度
static void ThreeHallangle_start(void)
{
	Hall_Three.Hall_State = GET_HALLState_debounce_dynamic();
	//根据霍尔初始状态给一个大概电角度
	//因为一个霍尔状态所代表的一个扇区有60度，此时不知道转子的具体角度，于是用扇区中间角度来估计作为转子初始角度，这样误差比较小
	switch (Hall_Three.Hall_State )
	{
		case 0x5:         
		{
			//Hall_Three.Predict_eleangle = HALL_0_PHASE_SHIFT +5462 + Hall_Three.Initial_eleangle; 
		    // 5462=30度在启动的几个霍尔状态中速度较低在二个霍尔状态中间角度控制 

			Hall_Three.Predict_eleangle = HALL_1_5_eleAngle + HALL_5_eleAngleWitdh/2;
		}
		break;
		case 0x1:
		{
			//Hall_Three.Predict_eleangle = HALL_300_PHASE_SHIFT + 5462 + Hall_Three.Initial_eleangle;
			Hall_Three.Predict_eleangle = HALL_3_1_eleAngle + HALL_1_eleAngleWitdh/2;
		}
		break;
		case 0x3:
		{
			//Hall_Three.Predict_eleangle = HALL_240_PHASE_SHIFT + 5462 + Hall_Three.Initial_eleangle;
			Hall_Three.Predict_eleangle = HALL_2_3_eleAngle + HALL_3_eleAngleWitdh/2;
		}
		break;
		case 0x2:
		{
			//Hall_Three.Predict_eleangle = HALL_180_PHASE_SHIFT + 5462  + Hall_Three.Initial_eleangle;
			Hall_Three.Predict_eleangle = HALL_6_2_eleAngle + HALL_2_eleAngleWitdh/2;
		}
		break;
		case 0x6:
		{
			//Hall_Three.Predict_eleangle = HALL_120_PHASE_SHIFT + 5462 + Hall_Three.Initial_eleangle;
			Hall_Three.Predict_eleangle = HALL_4_6_eleAngle + HALL_6_eleAngleWitdh/2;
		}
		break;
		case 0x4:
		{
			//Hall_Three.Predict_eleangle = HALL_60_PHASE_SHIFT + 5462 + Hall_Three.Initial_eleangle;
			Hall_Three.Predict_eleangle = HALL_5_4_eleAngle + HALL_4_eleAngleWitdh/2;
		}
		break;

		default:
		{
			Hall_Three.Predict_eleangle = 0;
		}
		break;
	}
	if( Hall_Three.Predict_eleangle >= 65536){
		Hall_Three.Predict_eleangle -= 65536;
	}
	if( Hall_Three.Predict_eleangle < 0){
		Hall_Three.Predict_eleangle += 65536;
	}
}
 
uint8_t GET_HALLState_debounce_dynamic(void)
{
    static uint8_t stable0 = 0, stable1 = 0, stable2 = 0;
    static uint16_t cnt0 = 0, cnt1 = 0, cnt2 = 0;
    uint8_t s0, s1, s2;
    uint8_t hall_code;

    // 更快读取三位 （直接读取 IDR）
//    uint16_t idr = GPIOC->IDR;
//    s0 = ( (idr & (1U<<13)) != 0 ) ? 1 : 0;
//    s1 = ( (idr & (1U<<14)) != 0 ) ? 1 : 0;
//    s2 = ( (idr & (1U<<15)) != 0 ) ? 1 : 0;
	  s0 = (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13)==1) ? 1 : 0;
	  s1 = (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_14)==1) ? 1 : 0;
	  s2 = (HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15)==1) ? 1 : 0;
		
	  uint16_t target_cnt = 1; //Hall_Three.HALLState_Filter_Count; // TBD:从动态计算拿到的值
    if (s0 == stable0) cnt0 = 0; else { if (++cnt0 >= target_cnt) { stable0 = s0; cnt0 = 0; } }
    if (s1 == stable1) cnt1 = 0; else { if (++cnt1 >= target_cnt) { stable1 = s1; cnt1 = 0; } }
    if (s2 == stable2) cnt2 = 0; else { if (++cnt2 >= target_cnt) { stable2 = s2; cnt2 = 0; } }

    Hall_Three.HallUVW[0] = stable0;
    Hall_Three.HallUVW[1] = stable1;
    Hall_Three.HallUVW[2] = stable2;

    hall_code = (stable0<<2) + (stable1<<1) + (stable2<<0);
    if (hall_code == 0 || hall_code == 7) {
        hall_code = Hall_Three.OldHall_State;
    } 
    return hall_code;
}


//noinline 能防止编译器把该函数直接内联到 Flash 里的调用点，确保它在 SRAM 段里。
//__attribute__((section("InRamCode"))) __attribute__((noinline))
uint32_t ThreeHallanglecale(void)  // 一个PWM周期执行一次
{
	/****** 1、获取HALL扇区状态  ******/
	//Hall_Three.Hall_State = get_hallValue();
    Hall_Three.Hall_State = GET_HALLState_debounce_dynamic();

	/*******  2、进行角度校准  *******/
	/* 如果本次三相霍尔状态和上次不同,进行角度校准,这里校准时可以加一个补偿值，
		补偿值根据速度动态改变。举例来说，若电机转速为5000转，每0.1ms转过0.1个扇区；
		如果HALL去抖判断为2次，则识别到变化时实际电角度可能已超过校准值0-0.1*2个扇区，
		此时应在校准值基础上加0.1扇区的补偿，使角度误差由0-0.1*2变为-0.1到0.1扇区；
		随着速度和去抖量的不同，补偿值也应动态改变，一般速度越小补偿值越小 */
	if (Hall_Three.Hall_State != Hall_Three.OldHall_State)
	{
		Hall_Three.HallLX_State = Hall_Three.Hall_State + (Hall_Three.OldHall_State << 4);
        Hall_Three.Move_OldState = Hall_Three.Move_State;

        //TBD: 动态计算校准量补偿值，补偿值恒为正值

		switch (Hall_Three.HallLX_State)  // 判断状态切换，正转和反转各有六种情况，正转增加补偿值反转减少补偿值
		{
			case 0x45:
			{
				Hall_Three.Move_State = HALL_Motor_REVERSAL;    // 反转（顺时针），偏转+60度电角度  此时应该在基础上减去补偿量
				// Hall_Three.Calibra_eleangle = Hall_Three.Initial_eleangle + HALL_60_PHASE_SHIFT -
				// 								Hall_Three.Calibra_Compensa_eleangle;
				Hall_Three.Calibra_eleangle = HALL_5_4_eleAngle - Hall_Three.Calibra_Compensa_eleangle;
				break;
			}
			case 0x15:
			{
				Hall_Three.Move_State = HALL_Motor_FORWARD;     // 正转（逆时针） 偏转+0度  在基础上加上补偿量
				// Hall_Three.Calibra_eleangle = Hall_Three.Initial_eleangle + HALL_0_PHASE_SHIFT +
				// 								Hall_Three.Calibra_Compensa_eleangle;
				Hall_Three.Calibra_eleangle = HALL_1_5_eleAngle + Hall_Three.Calibra_Compensa_eleangle;
				break;
			}
			case 0x51:
			{
				Hall_Three.Move_State = HALL_Motor_REVERSAL;      // 正转（0度）
				// Hall_Three.Calibra_eleangle = Hall_Three.Initial_eleangle + HALL_0_PHASE_SHIFT -
				// 								Hall_Three.Calibra_Compensa_eleangle;
				Hall_Three.Calibra_eleangle = HALL_1_5_eleAngle - Hall_Three.Calibra_Compensa_eleangle;
				break;
			}
			case 0x31:
			{
				Hall_Three.Move_State = HALL_Motor_FORWARD;     // 反转 (+300度)
				// Hall_Three.Calibra_eleangle = Hall_Three.Initial_eleangle + HALL_300_PHASE_SHIFT +
				// 								Hall_Three.Calibra_Compensa_eleangle;
				Hall_Three.Calibra_eleangle = HALL_3_1_eleAngle + Hall_Three.Calibra_Compensa_eleangle;
				break;
			}
			case 0x13:                                        
			{
				Hall_Three.Move_State = HALL_Motor_REVERSAL;     //   +300度 
				// Hall_Three.Calibra_eleangle = Hall_Three.Initial_eleangle + HALL_300_PHASE_SHIFT -
				// 												Hall_Three.Calibra_Compensa_eleangle;
				Hall_Three.Calibra_eleangle = HALL_3_1_eleAngle - Hall_Three.Calibra_Compensa_eleangle;
				break;
			}
			case 0x23:                 
			{
				Hall_Three.Move_State = HALL_Motor_FORWARD;     //  +240度
				// Hall_Three.Calibra_eleangle = Hall_Three.Initial_eleangle + HALL_240_PHASE_SHIFT +
				// 								Hall_Three.Calibra_Compensa_eleangle;
				Hall_Three.Calibra_eleangle = HALL_2_3_eleAngle + Hall_Three.Calibra_Compensa_eleangle;
				break;
			}
			case 0x32:
			{
				Hall_Three.Move_State = HALL_Motor_REVERSAL;     //  +240度
				// Hall_Three.Calibra_eleangle = Hall_Three.Initial_eleangle + HALL_240_PHASE_SHIFT -
				// 												Hall_Three.Calibra_Compensa_eleangle;
				Hall_Three.Calibra_eleangle = HALL_2_3_eleAngle - Hall_Three.Calibra_Compensa_eleangle;
				break;
			}
			case 0x62:
			{
				Hall_Three.Move_State = HALL_Motor_FORWARD;     //  +180度
				// Hall_Three.Calibra_eleangle = Hall_Three.Initial_eleangle + HALL_180_PHASE_SHIFT +
				// 								Hall_Three.Calibra_Compensa_eleangle;
				Hall_Three.Calibra_eleangle = HALL_6_2_eleAngle + Hall_Three.Calibra_Compensa_eleangle;
				break;
			}
			case 0x26:
			{
				Hall_Three.Move_State = HALL_Motor_REVERSAL;      //  +180度
				// Hall_Three.Calibra_eleangle = Hall_Three.Initial_eleangle + HALL_180_PHASE_SHIFT -
				// 												Hall_Three.Calibra_Compensa_eleangle;
				Hall_Three.Calibra_eleangle = HALL_6_2_eleAngle - Hall_Three.Calibra_Compensa_eleangle;
				break;
			}
			case 0x46:
			{
				Hall_Three.Move_State = HALL_Motor_FORWARD;      //  +120度
				// Hall_Three.Calibra_eleangle = Hall_Three.Initial_eleangle + HALL_120_PHASE_SHIFT +
				// 								Hall_Three.Calibra_Compensa_eleangle;
				Hall_Three.Calibra_eleangle = HALL_4_6_eleAngle + Hall_Three.Calibra_Compensa_eleangle;
				break;
			}
			case 0x64:
			{
				Hall_Three.Move_State = HALL_Motor_REVERSAL;        //  +120度
				// Hall_Three.Calibra_eleangle = Hall_Three.Initial_eleangle + HALL_120_PHASE_SHIFT -
				// 												Hall_Three.Calibra_Compensa_eleangle;
				Hall_Three.Calibra_eleangle = HALL_4_6_eleAngle - Hall_Three.Calibra_Compensa_eleangle;
				break;
			}
			case 0x54:
			{
				Hall_Three.Move_State = HALL_Motor_FORWARD;       //  +60度
				// Hall_Three.Calibra_eleangle = Hall_Three.Initial_eleangle + HALL_60_PHASE_SHIFT +
				// 												Hall_Three.Calibra_Compensa_eleangle;
				Hall_Three.Calibra_eleangle = HALL_5_4_eleAngle + Hall_Three.Calibra_Compensa_eleangle;
				break;
			}
			default:
			{
				// 未匹配状态，不做处理
				break;
			}
		}
		if(Hall_Three.Calibra_eleangle >= 65536) Hall_Three.Calibra_eleangle -= 65536;
		else if(Hall_Three.Calibra_eleangle < 0) Hall_Three.Calibra_eleangle += 65536;

		/* 3、更新平均电角速度、电角加速度、预测点补偿值、预测电角度 */

        //如果方向突然变换，清空Speed_count_buffer、num，置位Speed_Start_Flag，重新计算速度
        if(Hall_Three.Move_OldState != Hall_Three.Move_State)  //如果方向变换了
		{
			Hall_Three.Speed_Start_Flag = 1;  //置位电机启动标志位
			Hall_Three.Speed_count_num = 0;
			Hall_Three.Speed_count_buffer_index = 0;  //清空缓存区索引
			for(int i=0;i < WBuffer_MAX_SIZE;i++)
			{
				Hall_Three.Speed_count_buffer[i] = 0;   //初始化每个扇区的计数值
			}
		}

        //由于HALL安装有误差，因此不是每个扇区都是严格的60度，可能这个扇区为90度那个扇区为30度，因此我们需要对每个扇区的FOC中断计数进行滤波
		Hall_Three.Speed_count = _IQ10mpy(HallK1_Count, Hall_Three.Speed_count) + \
	                             _IQ10mpy(HallK2_Count,  Hall_Three.Speed_Oldcount);
		Hall_Three.Speed_Oldcount = Hall_Three.Speed_count;		

		Hall_Three.Speed_count_num -= Hall_Three.Speed_count_buffer[Hall_Three.Speed_count_buffer_index];
		Hall_Three.Speed_count_buffer[Hall_Three.Speed_count_buffer_index] = Hall_Three.Speed_count;
		Hall_Three.Speed_count_num += Hall_Three.Speed_count_buffer[Hall_Three.Speed_count_buffer_index];
		Hall_Three.Speed_count_buffer_index++;
		if(Hall_Three.Speed_Start_Flag == 1)  //电机启动期间速度计算（启动时候六个扇区都没有填满值，启动前6次有多少个数据就除多少个）
		{
			Hall_Three.Speed_Average_Oldcount = Hall_Three.Speed_Average_count;
			Hall_Three.Speed_Average_count=(Hall_Three.Speed_count_num/Hall_Three.Speed_count_buffer_index);   //更新平均电角速度Count计数

			if(Hall_Three.Speed_count_buffer_index >= WBuffer_MAX_SIZE)//<WBuffer_MAX_SIZE此处为6
			{
				Hall_Three.Speed_Start_Flag = 0;//<清除电机启动标志位
				Hall_Three.Speed_count_buffer_index = 0;
			}
		}
		else    ///滑动均值滤波（完成启动，缓存区都有6个数据，往后都除以6求平均）
		{
			Hall_Three.Speed_Average_Oldcount = Hall_Three.Speed_Average_count;
			Hall_Three.Speed_Average_count=(Hall_Three.Speed_count_num/WBuffer_MAX_SIZE);	
			if(Hall_Three.Speed_count_buffer_index >= WBuffer_MAX_SIZE)
			{
				Hall_Three.Speed_count_buffer_index=0;
			}
		}
		
		/*******  3、更新平均电角速度、电角加速度、预测点补偿值、预测电角度 *******/	
		if(Hall_Three.Speed_Average_count != 0){

				Hall_Three.Speed_Average_eleangle = 10923/Hall_Three.Speed_Average_count;    //<10923UL是60°弧度的整型定标
		}
		else{
			Hall_Three.Speed_Average_eleangle = 0;
		}
		if(((Hall_Three.Speed_Oldcount - Hall_Three.Speed_count) != 0)&&(Hall_Three.Speed_count != 0)){
			Hall_Three.Speed_Increase_eleangle = 10923/(Hall_Three.Speed_Average_Oldcount - Hall_Three.Speed_Average_count)/
			                                      Hall_Three.Speed_Average_count; 
		}
		else{
			Hall_Three.Speed_Increase_eleangle = 0;
		}
		 
		/*TBD: 预测点补偿值是什么意思呢？当校准点触发时，此时实际电角度一定和校准点电角度一致（大前提），这个时候将预测电角度
			和此时的校准点电角度做差然后除以上一个扇区经过的FOC次数，得出预测点补偿值，正转：如果补偿值为负就说明上一个扇区预测
			少了，下一个扇区预测要减去本补偿值（减去负值为增加）；为正就说明预测多了，下一个扇区预测电角度同样要减去本补偿值；
			反转：如果补偿值为负就说明上一个扇区预测多了，下一个扇区预测要减去上本补偿值；为正就说明预测少了，
			下一个扇区预测电角度同样要减去本补偿值；    总之不管正转反转，预测点角度时就是要减去预测点补偿值*/
		Hall_Three.Predict_Compensa_eleangle = Hall_Three.Predict_eleangle - Hall_Three.Calibra_eleangle;

		//当电机预测的电角度与校准点差大于某个度数(取60°一般合理，因为超过60°就已经跨SVPWM扇区了，是不可接受的）
		//就说明此时预测有严重偏差，将当前补偿值归零，下次重新计算补偿值；
		if(Hall_Three.Predict_Compensa_eleangle >= 10923)     //10923为60°电角度的整型值
		{
			Hall_Three.Predict_Compensa_eleangle = 0;  //预测点补偿值归零
		}
		else if (Hall_Three.Predict_Compensa_eleangle <= -10923)
		{
			Hall_Three.Predict_Compensa_eleangle = 0;
		}
		else                                          //计算预测点补偿值
		{
			Hall_Three.Predict_Compensa_eleangle =(Hall_Three.Predict_Compensa_eleangle / Hall_Three.Speed_count);  
		}

		//算出补偿值后再更新此时的预测电角度 = 校准点电角度
		Hall_Three.Predict_eleangle = Hall_Three.Calibra_eleangle;
		Hall_Three.Speed_count = 0;   //将扇区计数清零，经过下一个扇区时重新计数				 						    	 
    }
			
	/*******  4、堵转判断、预测、限幅 *******/		
	else //if (Hall_Three.Hall_State == Hall_Three.OldHall_State)    //如果本次三相霍尔状态和上次相同，说明还是在同一个电角度扇区中
	{
		Hall_Three.Speed_count++;
//		if (Hall_Three.Speed_count >= (Hall_Three.Locked_Time / Hall_Three.Speed_PWM_Time))   //待在一个扇区超过堵转时间
//		{
//			ThreeHallPara_init();      //TBD:把堵转判断和电位器对于电机启停状态的控制结合起来，堵转后需要电位器归零再启动
//		}

        /*****通过预测的电角速度进行插值来进行电角度预测*****/
		int32_t Limit_Judge_Differ;   //限幅判断差值
		if(Hall_Three.Move_State == HALL_Motor_FORWARD)
		{
            Hall_Three.Predict_eleangle += Hall_Three.Speed_Predict_Add_eleangle ;//- Hall_Three.Predict_Compensa_eleangle;
			/***限幅的意思是和上一次校准点电角度（不加校准点补偿值）不能超过一个扇区大小（HALL安装无误差时），毕竟如果超过了一个扇区的大小，
		    而前面校准点没有更新，说明此次预测的电角度肯定是有问题的（一切以校准点为准），比实际电角度要多，这里就做一个限幅。
			但是考虑到HALL安装误差，因此最终认为输出的实际电角度肯定不能超过上一次校准点电角度+1.3个扇区电角度****/
			
		     Limit_Judge_Differ = Hall_Three.Predict_eleangle - \
			                             (Hall_Three.Calibra_eleangle - Hall_Three.Calibra_Compensa_eleangle);
			 if(Limit_Judge_Differ >= 65536) Limit_Judge_Differ -= 65536;
			 else if(Limit_Judge_Differ < 0) Limit_Judge_Differ += 65536;
             if(Limit_Judge_Differ >= Hall_Three.HALL_eleAngleWitdh[Hall_Three.Hall_State]*11/10)  //如果超过了1.1个扇区的大小
			     Hall_Three.Predict_eleangle = (Hall_Three.Calibra_eleangle - Hall_Three.Calibra_Compensa_eleangle) 
				                               + Hall_Three.HALL_eleAngleWitdh[Hall_Three.Hall_State]*11/10;
		}
		else if(Hall_Three.Move_State == HALL_Motor_REVERSAL)
		{
			Hall_Three.Predict_eleangle -= Hall_Three.Speed_Predict_Add_eleangle ;//- Hall_Three.Predict_Compensa_eleangle;  
			
			Limit_Judge_Differ = (Hall_Three.Calibra_eleangle + Hall_Three.Calibra_Compensa_eleangle) -\
			                      Hall_Three.Predict_eleangle;
			if(Limit_Judge_Differ >= 65536) Limit_Judge_Differ -= 65536;
			else if(Limit_Judge_Differ < 0) Limit_Judge_Differ += 65536;
            if(Limit_Judge_Differ >= Hall_Three.HALL_eleAngleWitdh[Hall_Three.Hall_State]*11/10)  //如果超过了1.1个扇区的大小
			    Hall_Three.Predict_eleangle = (Hall_Three.Calibra_eleangle + Hall_Three.Calibra_Compensa_eleangle) 
				                             - Hall_Three.HALL_eleAngleWitdh[Hall_Three.Hall_State]*11/10;					  
		}
	}
    Hall_Three.OldHall_State = Hall_Three.Hall_State ;

    //锁相环PLL滤波
   Hall_Three.Predict_eleangle = Hall_PLL_filter(&Hall_Three);
	//最终输出限幅	 
    if(Hall_Three.Predict_eleangle >= 65536) Hall_Three.Predict_eleangle -= 65536;
	if(Hall_Three.Predict_eleangle < 0) Hall_Three.Predict_eleangle += 65536;
   
	return Hall_Three.Predict_eleangle;
}


int32_t Hall_PLL_filter(Hall *Hall_Three)
{
	#define SpeedPllBandwidth 250.0f   //rad/s
	#define SpeedPllKp (int16_t)(2 *SpeedPllBandwidth)//2*Wn*阻尼比 这里阻尼比为1
	#define SpeedPllKi (int16_t)(SpeedPllBandwidth * SpeedPllBandwidth/FOC_Frequency)// Wn*Wn*TS
	#define OmegaToTheta (int16_t)(16384*32768.0/FOC_Frequency/PI)

	Hall_Three->HallAngleSin_Cos.IQAngle = Hall_Three->Predict_eleangle;  //从0-2pi映射到0-65535
	IQSin_Cos_Cale((p_IQSin_Cos)&Hall_Three->HallAngleSin_Cos);

    Hall_Three->PLLAngleSin_Cos.IQAngle = Hall_Three->PLL_Theta;
	IQSin_Cos_Cale((p_IQSin_Cos)&Hall_Three->PLLAngleSin_Cos);

  Hall_Three->PLL_DeltTheta = (Hall_Three->HallAngleSin_Cos.IQSin * Hall_Three->PLLAngleSin_Cos.IQCos >> 15) - \
	                            (Hall_Three->HallAngleSin_Cos.IQCos * Hall_Three->PLLAngleSin_Cos.IQSin >> 15);
	Hall_Three->PLL_DeltTheta_Sum += Hall_Three->PLL_DeltTheta;

	Hall_Three->PLL_Omega = (SpeedPllKp * Hall_Three->PLL_DeltTheta >> 15) + \
	                            (SpeedPllKi * Hall_Three->PLL_DeltTheta_Sum >> 15);
	Hall_Three->PLL_Theta += (Hall_Three->PLL_Omega* OmegaToTheta >> 14);

	Hall_Three->PLL_Omega_filter += ((Hall_Three->PLL_Omega - Hall_Three->PLL_Omega_filter) >> 2);
	
	//最终输出限幅	 
  if(Hall_Three->PLL_Theta >= 65536) Hall_Three->PLL_Theta -= 65536;
	if(Hall_Three->PLL_Theta < 0) Hall_Three->PLL_Theta += 65536;
	return Hall_Three->PLL_Theta;
}

/**
 * @brief            锁相环法PLL核心程序(Kp与Ki根据电角速度自适应)
 * @param    hall    霍尔位置传感器结构体对象
 * @function         浮点运算直接求弧度，实现了锁相环法;
					 
 * @warnings         ！！！注意：速度用插值法速度，PLL估算出来的速度不能引入环路
 */
int32_t hall_pll_filter(Hall *Hall_Three)
{
	static int32_t error = 0;

	error = Hall_Three->Predict_eleangle - Hall_Three->pll_theta;

	/***********************电角度从0到2PI往复，误差相应要处理 **************************/
	/***********************要让 error 表示最短旋转方向与幅度，就把差值规范化为 −𝜋<𝑒𝑟𝑟𝑜𝑟≤𝜋，这样 error 始终表示最小角差
	 *                    （并带正确符号），PLL 就能稳定、以最短路径收敛 **********************/
	if(error >= 32768)
	{
		error -= 65536;
	}
	else if(error <= -32768)
	{
		error += 65536;
	}
	
	//角度的积分就是速度
	Hall_Three->pll_omega += (Hall_Three->pll_ki*error/(1000000/Hall_Three->Speed_PWM_Time));   //把ki的Ts在这里补上，减小计算精度丢失
	/*************************锁相环法电角速度的限幅 ****************************/
	/*pll_omega结构体成员的单位不是rad/s，而是一个FOC中断会经过多少的电角度*/
	/*笔者的实验电机2对极，额定转速3000RPM*/
	/*1310的选取背景是：把3000RPM(额定转速)转换为一个FOC中断会走多少电角度（0-360度换算为0-65535）单位，并乘以极对数*/
	/*比如极对数2乘以3000RPM约等于100转/s，又或者10KHZFOC中断下为 100*65536/10000 = 655 */
	/*有时候用到弱磁可以突破额定转速到5000RPM，所以给定一个较大的值，我这里直接655乘以2倍左右*/
	if(Hall_Three->pll_omega > 1310)
	{
		Hall_Three->pll_omega = 1310;
	}
	else if(Hall_Three->pll_omega < -1310)
	{
		Hall_Three->pll_omega = -1310;
	}
	/*************************锁相环法电角速度的限幅 ****************************/
	Hall_Three->pll_theta += Hall_Three->pll_omega + (Hall_Three->pll_kp * error /1000);  //把kp的/1000在这里补上，减小计算精度丢失

	/*************************电角度从0到2PI往复 ****************************/
	if(Hall_Three->pll_theta >= 65536)
	{
		Hall_Three->pll_theta -= 65536;
	}
	if(Hall_Three->pll_theta < 0)
	{
		Hall_Three->pll_theta += 65536;
	}
	/*************************电角度从0到2PI往复 ****************************/
	
	/*不能自己参照自己   wn^2 = (根号5-2)*wc^2  wc = 插值法估算的角加速度*/   
	/*改变Kp与Ki时，要使用插值法的速度  Ki = wn^2*Ts  Kp = 2*阻尼比*wn */
	/*插值法的速度估计非常重要，重要程度最高*/
	/*插值法的速度处理直接影响性能，包括启动，快速正反转，堵转恢复*/
	/*Hall_Three->Speed_Predict_Add_eleangle：插值法电角速度*/
	///<绝对不能用自己估算出来的速度，否则就是自己观测自己，控制系统不稳定

	//<0.236f约等于根号5-2
	Hall_Three->pll_ki = Hall_Three->Speed_Predict_Add_eleangle*Hall_Three->Speed_Predict_Add_eleangle*236/1000;
	//<阻尼比选0.707  2*0.707f=1.414f 根号0.236f 约等于 0.486f
	Hall_Three->pll_kp = 1414*486*Hall_Three->Speed_Predict_Add_eleangle/1000;

	return Hall_Three->pll_theta;
}
	
/**
 * @brief            锁相环法PLL核心程序(Kp与Ki根据电角速度自适应)
 * @param    hall    霍尔位置传感器结构体对象
 * @function         浮点运算直接求弧度，实现了锁相环法;
					 
 * @warnings         ！！！注意：速度用插值法速度，PLL估算出来的速度不能引入环路
 */
#define N_PI                       (3.14159265358979f)  ///<360°弧度制  
#define N2_PI                    (6.28318530717959f)  ///<360°弧度制  

int32_t  Speed_Predict_Add_eleangle;
int32_t hall_pll_filter_float(Hall *Hall_Three)
{
	static float theta_k= N2_PI/65536.0f;//<浮点数2PI用整型65536表示
	static float error=0;
	
	error = -((float)Hall_Three->Predict_eleangle*theta_k - Hall_Three->pll_theta);

	/***********************电角度从0到2PI往复，误差相应要处理 **************************/
	/***********************至于为什么这样处理，就交给读者思考啦！ **********************/
	if(error>=PI)
	{
		error -= N2_PI;
	}
	else if(error <= -PI)
	{
		error += N2_PI;
	}
	/***********************电角度从0到2PI往复，误差响应要处理 **************************/
	
	Hall_Three->pll_omega += (Hall_Three->pll_ki*error);
	Speed_Predict_Add_eleangle = (int32_t)(Hall_Three->pll_omega/(N2_PI/65536.0f)*((float)Hall_Three->Speed_PWM_Time/1000000.0f));
	/*************************锁相环法电角速度的限幅 ****************************/
	/*pll_omega结构体成员是弧度制(笔者整型和弧度制混用了，这方面代码没做到统一，是无法否认的缺点)*/
	/*笔者的实验电机2对极，额定转速3000RPM*/
	/*1236的选取背景是：把3000RPM(额定转速)转换为rad/s单位，并乘以极对数*/
	/*比如极对数2乘以3000RPM约等于618rad/s*/
	/*有时候用到弱磁可以突破额定转速到5000RPM，所以给定一个较大的值，我这里直接618乘以2倍左右*/
	if(Hall_Three->pll_omega>1236)
	{
		Hall_Three->pll_omega=1236;
	}
	else if(Hall_Three->pll_omega<-1236)
	{
		Hall_Three->pll_omega=-1236;
	}
	/*************************锁相环法电角速度的限幅 ****************************/
	Hall_Three->pll_theta+=(Hall_Three->pll_omega+Hall_Three->pll_kp*error)*((float)Hall_Three->Speed_PWM_Time/1000000.0f);
	
	/*************************电角度从0到2PI往复 ****************************/
	if(Hall_Three->pll_theta>N2_PI)
	{
		Hall_Three->pll_theta-=N2_PI;
	}
	if(Hall_Three->pll_theta<0)
	{
		Hall_Three->pll_theta+=N2_PI;
	}
  
	/*************************电角度从0到2PI往复 ****************************/
	
	/*不能自己参照自己*/
	/*改变Kp与Ki时，要使用插值法的速度*/
	/*插值法的速度估计非常重要，重要程度最高*/
	/*插值法的速度处理直接影响性能，包括启动，快速正反转，堵转恢复*/
	/*omega_inter：插值法电角速度*/
	///<绝对不能用自己估算出来的速度，否则就是自己观测自己，控制系统不稳定
    float omega_inter = (float)Hall_Three->Speed_Predict_Add_eleangle * theta_k/((float)Hall_Three->Speed_PWM_Time/1000000.0f); 
	Hall_Three->pll_ki=0.236f*omega_inter*omega_inter;//<0.236f约等于根号5-2
	arm_sqrt_f32(Hall_Three->pll_ki,&Hall_Three->pll_kp);
	Hall_Three->pll_kp*=1.414f;      //<阻尼比选0.707       2*0.707f=1.414f
	Hall_Three->pll_ki*= (float)Hall_Three->Speed_PWM_Time/1000000.0f;//<pll_Te是PLL法的执行时间，单位为s。放在FOC中断中其值就是1/f_FOC
	
	int32_t pll_theta = (int32_t)(Hall_Three->pll_theta/theta_k);
	if(pll_theta >= 65536)
	{
		pll_theta -= 65536;
	}
	if(pll_theta < 0)
	{
		pll_theta += 65536;
	}
	return pll_theta;
}


/**************************************************************************************/
#define HALL_DEB_MIN      1         // 最小去抖计数
#define HALL_DEB_MAX      20        // 最大去抖计数（避免过长延迟）

// 计算动态去抖计数的函数（根据速度动态改变去抖计数值）
void hall_dynamic_debounce_update(void)
{
    if (Hall_Three.Speed_RPM <= 100) {
        Hall_Three.HALLState_Filter_Count = HALL_DEB_MAX;
    } 
		else if(Hall_Three.Speed_RPM >= 3000){
       Hall_Three.HALLState_Filter_Count = HALL_DEB_MIN;
    } 
		else{
			Hall_Three.HALLState_Filter_Count = HALL_DEB_MAX - (HALL_DEB_MAX - HALL_DEB_MIN)*Hall_Three.Speed_RPM/3000;
    }
}

/**** 根据前面计算的平均电角速度、电角加速度，在这里进行电角速度预测以及RPM换算  
      注意这里的电角速度单位不是rad/s，而是一个FOC中断会经过多少的电角度****/
uint16_t Hall_Three_Speedcale(void)  // 1ms执行一次，计算电机磁场旋转速度
{
	Hall_Three.Speed_Predict_OldAdd_eleangle = Hall_Three.Speed_Predict_Add_eleangle;
	Hall_Three.Speed_Predict_Add_eleangle  =  Hall_Three.Speed_Average_eleangle + Hall_Three.Speed_Increase_eleangle;
	//一阶低通滤波
	Hall_Three.Speed_Predict_Add_eleangle = _IQ10mpy(HallK1_RPM, Hall_Three.Speed_Predict_Add_eleangle) + \
	                                        _IQ10mpy(HallK2_RPM,  Hall_Three.Speed_Predict_OldAdd_eleangle);
	if(Hall_Three.Move_State==0)   // Hall_Three.Move_State=0 停机，速度电角度为0，即速度为0
		Hall_Three.Speed_Predict_Add_eleangle = 0;
	if(Hall_Three.Speed_Predict_Add_eleangle < 0)  //电角速度限幅，一定为正
		Hall_Three.Speed_Predict_Add_eleangle = 0;

	//将预测的电角速度转换为机械速度RPM	   TBD：也可以用平均电角速度来转换，更平滑稳定
	/*这里最大角度2pi是一圈 65535，/65536是为把0-65535电角度转为0-1电角度圈数	*/
    Hall_Three.Speed_RPM = (uint16_t)(Hall_Three.Speed_Average_eleangle*(1000000/Hall_Three.Speed_PWM_Time*60)\
	                                  /65536/Hall_Three.Poles); 
    

    //根据速度重新计算堵转时间、滤波参数
	//模糊控制, 根据当前速度（即速度采样时间）动态更新速度一阶低通滤波系数
	if((Hall_Three.Speed_RPM >= 300)&&(Hall_Three.Speed_RPM < 4500))  //根据转速改变系数
	{
		//Hall_Three.Locked_Time = 1000000 - Hall_Three.Speed_RPM/300*30000;
		uint16_t T_s = Hall_Three.Speed_RPM/300*300*12/60;      //每300转改变一次滤波系数
		HallK1_Count = (uint16_t)((HallK1_Count_RC*1024)/(((uint16_t)1024/T_s)+ HallK1_Count_RC));
		HallK2_Count = 1024 - HallK1_Count;
	}
	/*因为采样时间为0.1ms，如果转速再大一些，系统响应就会有些滞后了，除非再改系数，
	这里没用公式算是因为Q6.10的采样时间已经不满足0.001s以内的精度了*/
	else if(Hall_Three.Speed_RPM >= 4500)  
	{
      //  Hall_Three.Locked_Time = 20000;
		HallK1_Count = 964;
		HallK2_Count = 60; 
	}
	else    //用初始系数
	{
		//Hall_Three.Locked_Time = 500000;
		HallK1_Count = 141;
		HallK2_Count = 883; 
	}
	
	hall_dynamic_debounce_update();

	return Hall_Three.Speed_RPM;  //返回电机转速
}
 


//===========================================================================
// No more.
//===========================================================================
		 


