#include "Missions.hpp"

#include "Parameters.hpp"
#include "semphr.h"
#include "Modes.hpp"

#define MissionParamVersion 1
#define MaxMissions 512
#define MissionsInParamGroupBit 5
#define MissionsInParamGroup (1<<MissionsInParamGroupBit)

static SemaphoreHandle_t MissionsSemphr = xSemaphoreCreateMutex();

static inline bool Lock_Missions( double TIMEOUT = -1 )
{
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake( MissionsSemphr , TIMEOUT_Ticks ) )
		return true;
	return false;
}
static inline void UnLock_Missions()
{
	xSemaphoreGive(MissionsSemphr);
}

//航点个数
static uint16_t MissionsCount = 0;
static uint16_t UploadingMissionsCount = 0;

//当前航点
static uint16_t CurrentMission = 0;
/*
	设置当前任务
	wpInd：当前任务序号
*/
bool setCurrentMission( uint16_t wpInd )
{
	if( MissionsCount==0 && wpInd==0 )
	{
		CurrentMission = 0;
		return false;
	}
	if( wpInd >= MissionsCount )
		return false;
	CurrentMission = wpInd;
	return true;
}
/*
	读取当前任务序号
*/
uint16_t getCurrentMissionInd() { return CurrentMission; }

/*
	获取航点个数
*/
uint16_t getMissionsCount()
{
	return MissionsCount;
}
/*
	获取正在上传航点个数
*/
uint16_t getUploadingMissionsCount()
{
	return UploadingMissionsCount;
}

/*
	清除所有航点任务
	TIMEOUT: 线程同步超时时间

	返回值：
	true:添加成功
	false:添加失败（航点过多或航点信息错误或参数表错误）
*/
bool clearMissions( double TIMEOUT )
{
	if( Lock_Missions(TIMEOUT) )
	{
		TruncateVolatileParamGroup( "Missions", 0 );
		
		CurrentMission = MissionsCount = UploadingMissionsCount = 0;
		
		UnLock_Missions();
		
	}
	else
		return false;
	
	CurrentWpInf initial_CurrentWpInf;
	initial_CurrentWpInf.CurrentWp[0] = 0;
	initial_CurrentWpInf.line_x = 0;
	initial_CurrentWpInf.line_y = 0;
	initial_CurrentWpInf.line_z = 0;
	initial_CurrentWpInf.line_fs = -1;
	UpdateParamGroup( "CurrentWp", (uint64_t*)&initial_CurrentWpInf, 0, sizeof(CurrentWpInf)/8 );
	return true;
}

/*
	添加航点任务
	wp_inf：航点信息
	st：是否写入存储器（只有当前实际航点数量为0才可以缓存不写入存储器）
	TIMEOUT: 线程同步超时时间

	返回值：
	true:添加成功
	false:添加失败（航点过多或航点信息错误或参数表错误）
*/
bool addMission( MissionInf wp_inf, bool st, double TIMEOUT )
{
	if( wp_inf.cmd == 0 )
		return false;
	if( MissionsCount > MaxMissions )
		return false;
	if( st==false && MissionsCount>0 )
		return false;
	if( st && MissionsCount!=UploadingMissionsCount )
		return false;
	
	if( Lock_Missions(TIMEOUT) )
	{
		WriteVolatileParamGroup( "Missions", &wp_inf, UploadingMissionsCount, 1, st );
		if(st)
		{
			++MissionsCount;
			UploadingMissionsCount = MissionsCount;
		}
		else
			++UploadingMissionsCount;
		
		UnLock_Missions();
		return true;
	}
	return false;
}

/*
	保存航点任务
	TIMEOUT: 线程同步超时时间

	返回值：
	true:保存成功
	false:保存失败（超时）
*/
bool saveMissions( double TIMEOUT )
{
	if( UploadingMissionsCount == 0 )
		return true;
	
	if( Lock_Missions(TIMEOUT) )
	{
		SaveVolatileParamGroup( "Missions" );
		MissionsCount = UploadingMissionsCount;
		
		UnLock_Missions();
		return true;
	}
	return false;
}

/*
	读取航点任务
	wp_ind：航点序号
	wp_inf：获取的航点信息
	TIMEOUT: 线程同步超时时间

	返回值：
	true:获取成功
	false:获取失败（无航点或参数表错误）
*/
bool ReadMission( uint16_t wp_ind, MissionInf* wp_inf, double TIMEOUT )
{
	if( wp_ind >= MissionsCount )
		return false;
	
	PR_RESULT res = ReadVolatileParamGroup( "Missions", wp_inf, wp_ind, 1, TIMEOUT );
	
	if( res == PR_ERR )
		return false;
	return true;
}

/*
	读取当前航点任务
	wp_inf：获取的航点信息
	ind：当前航点序号
	TIMEOUT: 线程同步超时时间

	返回值：
	true:获取成功
	false:获取失败（无航点或参数表错误）
*/
bool ReadCurrentMission( MissionInf* wp_inf, uint16_t* ind, double TIMEOUT )
{
	if( MissionsCount == 0 )
		return false;
	if( CurrentMission >= MissionsCount )
		return false;
	
	PR_RESULT res = ReadVolatileParamGroup( "Missions", wp_inf, CurrentMission, 1, TIMEOUT );

	if( res == PR_ERR )
		return false;
	if(ind!=0)
		*ind = CurrentMission;
	return true;
}

void init_Missions()
{	
	char WPGroupName[17];	
	MissionsCount = UploadingMissionsCount = 0;
	
	VolatileParamGroupRegister( "Missions", 1, sizeof(MissionInf), 50 );
	uint16_t missions_count;
	GetVolatileParamGroupParamCount( "Missions", &missions_count );
	MissionsCount = UploadingMissionsCount = missions_count;
	
	//读取模式配置
	ModeFuncCfg MFunc_cfg;
	ReadParamGroup( "MFunc", (uint64_t*)&MFunc_cfg, 0 );
	if( MFunc_cfg.RstWp0[0] == 0 )
	{
		//初始化CurrentMissionInd
		CurrentWpInf currentWpInf;
		ReadParamGroup( "CurrentWp", (uint64_t*)&currentWpInf, 0 );
		
		if( currentWpInf.CurrentWp[0] < missions_count )
			setCurrentMission(currentWpInf.CurrentWp[0]);
	}
	else
		setCurrentMission(0);
}