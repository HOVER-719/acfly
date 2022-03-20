#include "Basic.hpp"
#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "drv_Main.hpp"
#include "MS_Main.hpp"
#include "Parameters.hpp"
#include "FlashIO.h"
#include "Modes.hpp"
#include "MSafe.hpp"
#include "ctrl_Main.hpp"
#include "drv_PWMOut.hpp"
#include "AC_Math.hpp"
#include "Missions.hpp"
#include "ControlSystem.hpp"

#include "debug.hpp"

#if 1 //如果没有这段，则需要在target选项中选择使用USE microLIB

	__asm(".global __use_no_semihosting\n\t") ;//注释本行, 方法1
	extern "C"
	{
//		struct __FILE {
//		int handle;
//		};
//		std::FILE __stdout;

		void _sys_exit(int x)
		{
			x = x;
		}

		//__use_no_semihosting was requested, but _ttywrch was referenced, 增加如下函数, 方法2
		void _ttywrch(int ch)
		{
			ch = ch;
		}
		
		char *_sys_command_string(char *cmd, int len)
		{
				return 0;
		}
 
	}
#endif
	
//固件版本
#define Firmware_Version 16.3
	
void DriverInit_task(void* pvParameters)
{
	//初始化设备驱动
	init_drv_Main();
	//初始化解算系统
	init_MS_Main();
	init_Debug();
	init_Modes();
	init_MSafe();
	init_ControlSystem();	
	
	/*注册初始化参数*/
		struct
		{
			uint32_t calib_ESC[2];	//校准电调
			uint32_t calib_ESC_T[2];	//电调校准时间
			uint32_t boot_count[2];	//系统运行次数
			float Firmvare_Version[2];	//固件版本
		}init_cfg;
		init_cfg.calib_ESC[0] = 0;
		init_cfg.calib_ESC_T[0] = 3;
		init_cfg.boot_count[0] = 0;
		init_cfg.Firmvare_Version[0] = 16.3;
		MAV_PARAM_TYPE param_types[] = {
			MAV_PARAM_TYPE_UINT32 ,	//校准电调
			MAV_PARAM_TYPE_UINT32 ,	//电调校准时间
			MAV_PARAM_TYPE_UINT32 , //系统运行次数
			MAV_PARAM_TYPE_REAL32   //固件版本
			
		};
		SName param_names[] = {
			"Init_CalibESC",	//校准电调
			"Init_CalibESC_T",//电调校准时间
			"Init_Boot_Count",//系统运行次数
			"Init_Firmware_V" //固件版本
		};
		ParamGroupRegister( "Init", 1, 4, param_types, param_names, (uint64_t*)&init_cfg );	
		/*注册初始化参数*/
	
	//完成初始化
	//完成后不能再进行初始化操作
	while( getInitializationCompleted() == false )
	{
		setInitializationCompleted();
		os_delay(0.1);
	}
	
	uint8_t uat_type[8];
	if( ReadParam( "AC_UAVType", 0, 0, (uint64_t*)uat_type, 0 ) == PR_OK )
	{	//获取主电机数量
		set_MainMotorCount(UAV_MainMotorCount(uat_type[0]));
	}
	
	/*读取初始化参数进行初始化操作*/
		//读初始化参数
		ReadParamGroup( "Init", (uint64_t*)&init_cfg, 0 );
	
		//校准电调
		if( init_cfg.calib_ESC[0] == 21586 )
		{
			MainMotor_PullUpAll();
			os_delay(init_cfg.calib_ESC_T[0]);
			MainMotor_PullDownAll();
			
			init_cfg.calib_ESC[0] = 0;
		}
		
		//系统重启次数加1
		init_cfg.boot_count[0]++;
	  	init_cfg.Firmvare_Version[0] = Firmware_Version;
		//重置初始化参数
		UpdateParamGroup( "Init", (uint64_t*)&init_cfg, 0, sizeof(init_cfg)/8 );
		
		//拉低电机输出
		MainMotor_PullDownAll();
	/*读取初始化参数进行初始化操作*/
	
	//飞行任务初始化
	init_Missions();
	
	//删除本任务
	vTaskDelete(0);
}
	
int main(void)
{	
	//初始化芯片时钟
	//时间基准等基础功能
  init_Basic();
			
	//创建初始化任务并进入任务调度器
	xTaskCreate( DriverInit_task , "Init" ,8192,NULL,3,NULL);
	vTaskStartScheduler();
	while(1);
}

extern "C" void HardFault_Handler()
{
	//错误中断拉低所有输出
	PWM_PullDownAll();
}

extern "C" void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{
	static char StackOvTaskName[20];
	strcpy( StackOvTaskName, (char*)pcTaskName );
}



