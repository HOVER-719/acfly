#include "Parameters.hpp"
#include <map>
#include "StorageSystem.hpp"
#include "FreeRTOS.h"
#include "semphr.h"
#include "Basic.hpp"
#include "mavlink.h"

using namespace std;

/*
	参数组格式：
		参数组版本version(0:1) 15:14为参数类型 
			0-不加入参数列表的普通参数组
			1-变长参数组（不能加入参数组）
			2-加入参数列表的普通参数组
		参数个数n(2:3)
		更新日期(4:7)
		n个参数(32字节一个）加入参数列表情况
		{
			参数类型MAV_PARAM_TYPE(8+n*32:9+n*32)
			保留(10+n*32:15+n*32)
			单独参数名(16+n*32:31+n*32)
			参数值(32+n*32:39+n*32)
		}
		n个参数(16字节一个）不加入参数列表情况
		{
			参数类型MAV_PARAM_TYPE(8+n*16:9+n*16)
			保留(10+n*16:15+n*16)
			参数值(16+n*16:23+n*16)
		}
*/

//内存普通参数组数据
struct ParamGroup
{
	//参数组名称
	SName name;
	//是否为新参数
	bool is_new;
	//参数组读访问互斥信号量
	SemaphoreHandle_t ReadSemphr;
	//参数组数据指针
	//ParamGroupInf+n*ParamData
	void* data;
};
//内存可变参数组数据
struct VolatileParamGroup:ParamGroup
{
	uint16_t params_count;	//当前参数个数
//	uint16_t ram_params;	//内存中可存放的参数个数
	uint16_t ram_param_start;	//内存中的参数起始序号
	uint16_t ram_param_count;	//内存中的参数个数
	bool ram_updated;
};

//一般参数组头
struct ParamGroupInf
{
	uint16_t version;
	uint16_t params_count;
	uint32_t update_date;
}__attribute__((__packed__));
//不定长参数组头
struct VolatileParamGroupInf
{
	uint16_t version;
	uint16_t param_length;
	uint32_t update_date;
}__attribute__((__packed__));
//带参数名参数格式（出现在参数列表）
struct ParamData1
{
	uint16_t type;
	uint8_t resv1[6];
	SName name;
	uint64_t data;
}__attribute__((__packed__));
//无参数名参数格式（不在参数列表）
struct ParamData2
{
	uint64_t data;
}__attribute__((__packed__));
struct Param
{
	ParamGroup* group;
	ParamData1* data;
	uint32_t index;
};

//参数组写互斥锁
static SemaphoreHandle_t ParamGroup_Semphr;
//参数表写互斥锁
static SemaphoreHandle_t Params_Semphr;
//参数组表
static map<SName, ParamGroup*> ParamGroups;
//参数表
static map<SName, Param> GroupParameters;
//参数表长度
static uint32_t ParametersCount = 0;

/*
	可变长参数组注册（所有名称小于等于16字节）
	注意：仅可在参数表初始化完成前调用
	name：参数组名称
	param_version：参数版本
	param_length：单个参数长度
	ram_params_count：在ram中可存放的参数个数n

	返回值：
	PR_ERR：参数表初始化未完成
	PR_Existed: 参数表中已有同名参数
	PR_TimeOut：访问参数表操作超时
	PR_NewParam：注册的参数是新参数（存储器里没有）
	PR_OK：注册的参数在存储器已找到并更新了param
*/
PR_RESULT VolatileParamGroupRegister( SName name, uint16_t version, 
															uint16_t param_length, uint32_t ram_params_count )
{			
	if( param_length == 0 )
		return PR_ERR;
	if( ram_params_count==0 || ram_params_count>MAX_VOLATILE_PARAMS )
		return PR_ERR;
	
	//参数组长度
	uint32_t ParamGroup_length;
	//配置参数组版本及长度
	version &= 0x3f;
	version |= (1<<14);
	ParamGroup_length = sizeof(VolatileParamGroupInf) + param_length*ram_params_count;
		
	//申请内存（portBYTE_ALIGNMENT必须为8或其整数倍以保证64位浮点访问）
	uint8_t* newParamGroupData = new uint8_t[ParamGroup_length];
	
	PR_RESULT rt_res = PR_ERR;
	//获取存储的参数长度信息		
	uint32_t ram_ParamGroup_length;
	uint32_t stored_ParamGroup_length;
	char name_ch[17];
	name.get_CharStr(name_ch);
	SS_RESULT res = InternalStorage_GetFileSize( "Config", name_ch, &stored_ParamGroup_length );
	if( res==SS_OK && ( (stored_ParamGroup_length-sizeof(VolatileParamGroupInf))%param_length==0 ) )
	{	//参数已保存在存储器
		//获取保存的参数
		res = InternalStorage_ReadFile( "Config", name_ch, newParamGroupData, &ram_ParamGroup_length, 
				0, (ParamGroup_length < stored_ParamGroup_length)? ParamGroup_length : stored_ParamGroup_length );
		if( res==SS_OK )
		{
			VolatileParamGroupInf* inf = (VolatileParamGroupInf*)&(newParamGroupData[0]);
			if( inf->version==version && inf->param_length==param_length )
			{	//参数读取+校验成功
				rt_res = PR_OK;
			}
		}
	}		
	
	if( rt_res != PR_OK )
	{	//参数未保存在存储器
		//初始化参数
		RTC_TimeStruct RTCTime = Get_RTC_Time();
		uint32_t file_create_time =  ( (RTCTime.Year - 1980) << 25 ) 	|
																 ( RTCTime.Month << 21 ) 	|
																 ( RTCTime.Date << 16 ) 	|
																 ( RTCTime.Hours << 11 ) 	|
																 ( RTCTime.Minutes << 5 ) 	|
																 ( RTCTime.Seconds << 0 );
		VolatileParamGroupInf* inf = (VolatileParamGroupInf*)&(newParamGroupData[0]);
		inf->version = version;
		inf->param_length = param_length;
		inf->update_date = file_create_time;
		
		rt_res = PR_NewParam;
	}
	
	//初始化参数组
	VolatileParamGroup* newParamGroup = new VolatileParamGroup();
	newParamGroup->name = name;
	newParamGroup->ram_param_start = 0;
	newParamGroup->ram_param_count = ram_params_count;
	if( rt_res == PR_NewParam )
	{
		newParamGroup->is_new = true;
	}
	else
	{
		newParamGroup->is_new = false;
		newParamGroup->params_count = 
			(stored_ParamGroup_length - sizeof(VolatileParamGroupInf)) / param_length;
	}
	newParamGroup->ReadSemphr = xSemaphoreCreateMutex();	
	newParamGroup->data = newParamGroupData;
	newParamGroup->ram_updated = false;
	
	//参数组加入参数组表
	if( xSemaphoreTake( ParamGroup_Semphr, portMAX_DELAY ) == pdTRUE )
	{		
		//检测参数表是否已存在
		if( ParamGroups.find(name) != ParamGroups.end() )
		{
			delete[] newParamGroupData;
			delete newParamGroup;
			xSemaphoreGive(ParamGroup_Semphr);
			return PR_Existed;
		}
		
		ParamGroups.insert( 
			pair<SName, VolatileParamGroup*>( name, newParamGroup ) );
		
		xSemaphoreGive(ParamGroup_Semphr);
	}	

	return PR_OK;
}

/*
	参数组注册（所有名称小于等于16字节）
	注意：仅可在参数表初始化完成前调用
	name：参数组名称
	param_version：参数版本
	params_count：参数个数n

	param_types：参数类型*n
	param_names：参数名称*n（可以为NULL代表不需要加入参数列表）
	initial_params：参数初始值*n

	返回值：
	PR_ERR：参数表初始化未完成
	PR_Existed: 参数表中已有同名参数
	PR_TimeOut：访问参数表操作超时
	PR_NewParam：注册的参数是新参数（存储器里没有）
	PR_OK：注册的参数在存储器已找到并更新了param
*/
PR_RESULT ParamGroupRegister( SName name, uint16_t version, uint32_t params_count,
															const MAV_PARAM_TYPE param_types[], const SName param_names[], const uint64_t initial_params[] )
{	
	//是否需要加入参数列表
	bool addParameters = false;
	if( param_names != 0 )
		addParameters = true;
	
	//初始化完成后不允许添加参数表
	if( addParameters )
	{
		LockInitializationStatus();
		if( getInitializationCompleted() == true )
		{	//参数表初始化完成后不允许注册参数
			UnLockInitializationStatus();
			return PR_ERR;
		}
	}
				
	//参数组长度
	uint32_t ParamGroup_length;
	//配置参数组版本及长度
	if( addParameters )
	{
		//参数组版本
		version &= 0x3f;
		version |= (2<<14);
		ParamGroup_length = sizeof(ParamGroupInf) + sizeof(ParamData1)*params_count;
	}
	else
	{
		version &= 0x3f;
		version |= (0<<14);
		ParamGroup_length = sizeof(ParamGroupInf) + sizeof(ParamData2)*params_count;
	}
	//申请内存（portBYTE_ALIGNMENT必须为8或其整数倍以保证64位浮点访问）
	uint8_t* newParamGroupData = new uint8_t[ParamGroup_length];
	
	PR_RESULT rt_res = PR_ERR;
	//获取存储的参数长度信息			
	uint32_t stored_ParamGroup_length;
	char name_ch[17];
	name.get_CharStr(name_ch);
	SS_RESULT res = InternalStorage_GetFileSize( "Config", name_ch, &stored_ParamGroup_length );
	if( res==SS_OK && stored_ParamGroup_length==ParamGroup_length )
	{	//参数已保存在存储器
		//获取保存的参数
		InternalStorage_ReadFile( "Config", name_ch, newParamGroupData, &stored_ParamGroup_length );
		ParamGroupInf* inf = (ParamGroupInf*)&(newParamGroupData[0]);
		if( inf->version==version && params_count==inf->params_count )
			rt_res = PR_OK;
		if( rt_res==PR_OK && addParameters )
		{
			for( uint16_t i = 0 ; i < params_count ; ++i )
			{
				ParamData1* data = (ParamData1*)&(newParamGroupData[sizeof(ParamGroupInf)+i*sizeof(ParamData1)]);
				if( data->type!=param_types[i] || data->name!=param_names[i] )
				{
					rt_res = PR_ERR;
					break;
				}						
			}	
		}
	}
	
	if( rt_res != PR_OK )
	{	//参数未保存在存储器
		//初始化参数
		RTC_TimeStruct RTCTime = Get_RTC_Time();
		uint32_t file_create_time =  ( (RTCTime.Year - 1980) << 25 ) 	|
																 ( RTCTime.Month << 21 ) 	|
																 ( RTCTime.Date << 16 ) 	|
																 ( RTCTime.Hours << 11 ) 	|
																 ( RTCTime.Minutes << 5 ) 	|
																 ( RTCTime.Seconds << 0 );
		ParamGroupInf* inf = (ParamGroupInf*)&(newParamGroupData[0]);
		inf->version = version;
		inf->params_count = params_count;
		inf->update_date = file_create_time;
		if(addParameters)
		{	//加入参数表的参数需要名称
			for( uint16_t i = 0 ; i < params_count ; ++i )
			{
				ParamData1* data = (ParamData1*)&(newParamGroupData[sizeof(ParamGroupInf)+i*sizeof(ParamData1)]);
				data->type = param_types[i];
				data->name = param_names[i];
				if( initial_params != 0 )
					data->data = initial_params[i];
			}	
		}
		else
		{	//不加入参数表的参数没有名称信息
			for( uint16_t i = 0 ; i < params_count ; ++i )
			{
				ParamData2* data = (ParamData2*)&(newParamGroupData[sizeof(ParamGroupInf)+i*sizeof(ParamData2)]);
				if( initial_params != 0 )
					data->data = initial_params[i];
			}	
		}
		rt_res = PR_NewParam;
	}
	
	//初始化参数组
	ParamGroup* newParamGroup = new ParamGroup();
	newParamGroup->name = name;
	if( rt_res == PR_NewParam )
		newParamGroup->is_new = true;
	else
		newParamGroup->is_new = false;
	newParamGroup->ReadSemphr = xSemaphoreCreateMutex();
	newParamGroup->data = newParamGroupData;
	
	//参数组加入参数组表
	if( xSemaphoreTake( ParamGroup_Semphr, portMAX_DELAY ) == pdTRUE )
	{		
		//检测参数表是否已存在
		if( ParamGroups.find(name) != ParamGroups.end() )
		{
			delete[] newParamGroupData;
			delete newParamGroup;
			xSemaphoreGive(ParamGroup_Semphr);
			return PR_Existed;
		}
		
		ParamGroups.insert( 
			pair<SName, ParamGroup*>( name, newParamGroup ) );
		
		xSemaphoreGive(ParamGroup_Semphr);
	}	
	
	if(addParameters)
	{
		if( xSemaphoreTake( Params_Semphr, portMAX_DELAY ) == pdTRUE )
		{
			//参数加入参数列表
			for( uint16_t i = 0 ; i < params_count ; ++i )
			{
				Param param = { newParamGroup, (ParamData1*)&(newParamGroupData[sizeof(ParamGroupInf)+i*sizeof(ParamData1)]), ParametersCount };			
				GroupParameters.insert( pair<SName, Param>( param_names[i], param ) );
				++ParametersCount;
			}
			xSemaphoreGive(Params_Semphr);
		}	
	}
	
	//解锁
	if(addParameters)
		UnLockInitializationStatus();
		
	return PR_OK;
}

/*
	参数组注册（所有名称小于等于16字节）
	注意：此函数只能注册已在存储器中的参数
	注意：仅可在参数表初始化完成前调用
	name：参数组名称
	param_version：参数版本
	params_count：参数个数n

	param_names：参数名称*n（可以为NULL代表不需要加入参数列表）

	返回值：
	PR_ERR：参数表初始化未完成或参数在存储器中不存在
	PR_Existed: 参数表中已有同名参数
	PR_TimeOut：访问参数表操作超时
	PR_NewParam：注册的参数是新参数（存储器里没有）
	PR_OK：注册的参数在存储器已找到并更新了param
*/
PR_RESULT ParamGroupRegister( SName name, uint16_t version, uint32_t params_count, const SName param_names[] )
{	
	//是否需要加入参数列表
	bool addParameters = false;
	if( param_names != 0 )
		addParameters = true;
	
	//初始化完成后不允许添加参数表
	if( addParameters )
	{
		LockInitializationStatus();
		if( getInitializationCompleted() == true )
		{	//参数表初始化完成后不允许注册参数
			UnLockInitializationStatus();
			return PR_ERR;
		}
	}
				
	//参数组长度
	uint32_t ParamGroup_length;
	//配置参数组版本及长度
	if( addParameters )
	{
		//参数组版本
		version &= 0x3f;
		version |= (2<<14);
		ParamGroup_length = sizeof(ParamGroupInf) + sizeof(ParamData1)*params_count;
	}
	else
	{
		version &= 0x3f;
		version |= (0<<14);
		ParamGroup_length = sizeof(ParamGroupInf) + sizeof(ParamData2)*params_count;
	}
	//申请内存（portBYTE_ALIGNMENT必须为8或其整数倍以保证64位浮点访问）
	uint8_t* newParamGroupData = new uint8_t[ParamGroup_length];
	
	//立刻从存储器获取配置
	PR_RESULT rt_res = PR_ERR;
	//获取存储的参数长度信息			
	uint32_t stored_ParamGroup_length;
	char name_ch[17];
	name.get_CharStr(name_ch);
	SS_RESULT res = InternalStorage_GetFileSize( "Config", name_ch, &stored_ParamGroup_length );
	if( res==SS_OK && stored_ParamGroup_length==ParamGroup_length )
	{	//参数已保存在存储器
		//获取保存的参数
		InternalStorage_ReadFile( "Config", name_ch, newParamGroupData, &stored_ParamGroup_length );
		ParamGroupInf* inf = (ParamGroupInf*)&(newParamGroupData[0]);
		if( inf->version==version && params_count==inf->params_count )
			rt_res = PR_OK;
	}
	
	if( rt_res != PR_OK )
	{	//参数未保存在存储器
		//退出
		delete[] newParamGroupData;
		return PR_ERR;
	}
	
	//初始化参数组
	ParamGroup* newParamGroup = new ParamGroup();
	newParamGroup->name = name;
	if( rt_res == PR_NewParam )
		newParamGroup->is_new = true;
	else
		newParamGroup->is_new = false;
	newParamGroup->ReadSemphr = xSemaphoreCreateMutex();
	newParamGroup->data = newParamGroupData;
	
	//参数组加入参数组表
	if( xSemaphoreTake( ParamGroup_Semphr, portMAX_DELAY ) == pdTRUE )
	{
		//检测参数表是否已存在
		if( ParamGroups.find(name) != ParamGroups.end() )
		{
			delete[] newParamGroupData;
			xSemaphoreGive(ParamGroup_Semphr);
			return PR_Existed;
		}
		
		ParamGroups.insert( 
			pair<SName, ParamGroup*>( name, newParamGroup ) );
		
		xSemaphoreGive(ParamGroup_Semphr);
	}	
	
	if(addParameters)
	{
		if( xSemaphoreTake( Params_Semphr, portMAX_DELAY ) == pdTRUE )
		{
			//参数加入参数列表
			for( uint16_t i = 0 ; i < params_count ; ++i )
			{
				Param param = { newParamGroup, (ParamData1*)&(newParamGroupData[sizeof(ParamGroupInf)+i*sizeof(ParamData1)]), ParametersCount };			
				GroupParameters.insert( pair<SName, Param>( param_names[i], param ) );
				++ParametersCount;
			}
			xSemaphoreGive(Params_Semphr);
		}	
	}
	
	//解锁
	if(addParameters)
		UnLockInitializationStatus();
	
	return PR_OK;
}

static SemaphoreHandle_t Parameters_it_Mutex = xSemaphoreCreateMutex();
static map<SName, Param>::iterator Parameters_it = GroupParameters.end();
/*
	获取参数表参数个数
	注意：仅可在参数表初始化完成后调用
	count：返回参数个数
返回值：
	PR_ERR：未完成初始化
	PR_OK：读取成功
*/
PR_RESULT GetParametersCount( uint32_t* count )
{
	//参数表未完成初始化不允许读取参数
	if( getInitializationCompleted() == false )
		return PR_ERR;
	*count = ParametersCount;
	return PR_OK;
}
/*
	重置参数表读取迭代器
	注意：仅可在参数表初始化完成后调用
	count：返回参数个数
返回值：
	PR_ERR：未完成初始化
	PR_OK：操作成功
*/
PR_RESULT ResetParametersIterator()
{
	//参数表未完成初始化不允许读取参数
	if( getInitializationCompleted() == false )
		return PR_ERR;
	xSemaphoreTake(Parameters_it_Mutex,portMAX_DELAY);
		Parameters_it = GroupParameters.begin();
	xSemaphoreGive(Parameters_it_Mutex);
	return PR_OK;
}
/*
	递增迭代器
	注意：仅可在参数表初始化完成后调用
返回值：
	PR_ERR：未完成初始化或已经在末尾
	PR_OK：操作成功
*/
PR_RESULT ParameterIteratorMoveNext()
{
	//参数表未完成初始化不允许读取参数
	if( getInitializationCompleted() == false )
		return PR_ERR;
	xSemaphoreTake(Parameters_it_Mutex,portMAX_DELAY);
		if( Parameters_it == GroupParameters.end() )
		{
			xSemaphoreGive(Parameters_it_Mutex);			
			return PR_ERR;
		}
		++Parameters_it;
	xSemaphoreGive(Parameters_it_Mutex);
	return PR_OK;
}
/*
	读当前参数
	注意：仅可在参数表初始化完成后调用
	index：当前参数序号
	name：读取的参数名称
	type：读取的参数类型
	value：读取的参数的值
	is_new：参数是为新（不在存储器）
返回值：
	PR_ERR：未完成初始化或已经在末尾
	PR_TimeOut：访问参数表操作超时
	PR_OK：操作成功
*/
PR_RESULT ReadCurrentParameter( SName* name, uint32_t* index, MAV_PARAM_TYPE* type, uint64_t* value, bool* is_new, double TIMEOUT )
{
	//参数表未完成初始化不允许读取参数
	if( getInitializationCompleted() == false )
		return PR_ERR;
	
	Param param;
	xSemaphoreTake(Parameters_it_Mutex,portMAX_DELAY);
		if( Parameters_it == GroupParameters.end() )
		{
			xSemaphoreGive(Parameters_it_Mutex);
			return PR_ERR;
		}
		param = Parameters_it->second;		
	xSemaphoreGive(Parameters_it_Mutex);
	if( index != 0 )
		*index = param.index;
	
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake(param.group->ReadSemphr, TIMEOUT_Ticks) == pdTRUE )
	{
		if( name != 0 )
			*name = param.data->name;
		if( type != 0 )
			*type = (MAV_PARAM_TYPE)param.data->type;
		if( value != 0 )
			*value = param.data->data;
		if( is_new != 0 )
			*is_new = param.group->is_new;
		xSemaphoreGive(param.group->ReadSemphr);
		return PR_OK;
	}
	return PR_TimeOut;
}

/*
	读取指定名称参数
	注意：仅可在参数表初始化完成后调用
	name：参数名称
	index：读取的参数序号
	type：读取的参数类型
	value：要读取的参数的值
	is_new：参数是为新（不在存储器）

	返回值：
	PR_ERR：参数表中无此参数
	PR_TimeOut：访问参数表操作超时
	PR_OK：读取成功
*/
PR_RESULT ReadParam( SName name, uint32_t* index, MAV_PARAM_TYPE* type, uint64_t* value, bool* is_new, double TIMEOUT )
{
	//参数表未完成初始化不允许读取参数
	if( getInitializationCompleted() == false )
		return PR_ERR;
	
	map<SName, Param>::iterator it = GroupParameters.find(name);
	if( it == GroupParameters.end() )
		//无此参数退出
		return PR_ERR;
	Param param = it->second;
	
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake(param.group->ReadSemphr, TIMEOUT_Ticks) == pdTRUE )
	{
		if( index != 0 )
			*index = param.index;
		if( type != 0 )
			*type = (MAV_PARAM_TYPE)param.data->type;
		if( value != 0 )
			*value = param.data->data;
		if( is_new != 0 )
			*is_new = param.group->is_new;
		xSemaphoreGive(param.group->ReadSemphr);
		return PR_OK;
	}
	return PR_TimeOut;
}
/*
	读取指定序号参数（低效率）
	注意：仅可在参数表初始化完成后调用
	index：参数序号
	name：参数名称
	type：读取的参数类型
	value：要读取的参数的值
	is_new：参数是为新（不在存储器）

	返回值：
	PR_ERR：参数表中无此参数
	PR_TimeOut：访问参数表操作超时
	PR_OK：读取成功
*/
PR_RESULT ReadParam( uint32_t index, SName* name, MAV_PARAM_TYPE* type, uint64_t* value, bool* is_new, double TIMEOUT )
{
	//参数表未完成初始化不允许读取参数
	if( getInitializationCompleted() == false )
		return PR_ERR;
	if( index >= ParametersCount )
		return PR_ERR;
	
	map<SName, Param>::iterator it = GroupParameters.begin();
	for( ; it != GroupParameters.end(); ++it )
	{
		if( it->second.index == index )
			break;
	}
	if( it == GroupParameters.end() )
		//无此参数退出
		return PR_ERR;
	Param param = it->second;
	
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake(param.group->ReadSemphr, TIMEOUT_Ticks) == pdTRUE )
	{
		if( name != 0 )
			*name = param.data->name;
		if( type != 0 )
			*type = (MAV_PARAM_TYPE)param.data->type;
		if( value != 0 )
			*value = param.data->data;
		if( is_new != 0 )
			*is_new = param.group->is_new;
		xSemaphoreGive(param.group->ReadSemphr);
		return PR_OK;
	}
	return PR_TimeOut;
}

/*
	读取指定名称普通参数组
	注意：仅可在参数表初始化完成后调用
	name：参数组名称
	data：要读取的参数组参数的值

	返回值：
	PR_ERR：参数表中无此参数
	PR_TimeOut：访问参数表操作超时
	PR_OK：读取成功
*/
PR_RESULT ReadParamGroup( SName name, uint64_t data[], bool* is_new, double TIMEOUT )
{
	//参数表未完成初始化不允许读取参数
	if( getInitializationCompleted() == false )
		return PR_ERR;
	
	//获取参数组
	ParamGroup* group;
	if( xSemaphoreTake( ParamGroup_Semphr, portMAX_DELAY ) == pdTRUE )
	{
		map<SName, ParamGroup*>::iterator it = ParamGroups.find(name);
		if( it == ParamGroups.end() )
		{	//无此参数退出
			xSemaphoreGive(ParamGroup_Semphr);
			return PR_ERR;
		}
		group = it->second;
	
		xSemaphoreGive(ParamGroup_Semphr);
	}
		
	ParamGroupInf* group_inf = (ParamGroupInf*)group->data;
	if( group_inf->version & (1<<14) )
	{	//不为普通参数组退出
		return PR_ERR;
	}
	//参数类别
	bool is_param1 = ( group_inf->version & (1<<15) ) != 0;
	
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake(group->ReadSemphr, TIMEOUT_Ticks) == pdTRUE )
	{
		if( is_new != 0 )
			*is_new = group->is_new;
		for( uint16_t i = 0 ; i < group_inf->params_count ; ++i )
		{
			if( is_param1 )
			{
				ParamData1* param = (ParamData1*)&((uint8_t*)group->data)[sizeof(ParamGroupInf)+i*sizeof(ParamData1)];
				data[i] = param->data;
			}
			else
			{
				ParamData2* param = (ParamData2*)&((uint8_t*)group->data)[sizeof(ParamGroupInf)+i*sizeof(ParamData2)];
				data[i] = param->data;
			}
		}
		xSemaphoreGive(group->ReadSemphr);
		return PR_OK;
	}
	return PR_TimeOut;
}
/*
	读取指定名称普通参数组
	注意：仅可在参数表初始化完成后调用
	name：参数组名称
	data：要读取的参数组参数的值
	start：从序号start开始读取（0开始）
	read_count：读取数目

	返回值：
	PR_ERR：参数表中无此参数或读取的参数超出参数范围
	PR_TimeOut：访问参数表操作超时
	PR_OK：读取成功
*/
PR_RESULT ReadParamGroup( SName name, uint64_t data[], bool* is_new, uint16_t start, uint16_t read_count, double TIMEOUT )
{
	//参数表未完成初始化不允许读取参数
	if( getInitializationCompleted() == false )
		return PR_ERR;
	
	//获取参数组
	ParamGroup* group;
	if( xSemaphoreTake( ParamGroup_Semphr, portMAX_DELAY ) == pdTRUE )
	{
		map<SName, ParamGroup*>::iterator it = ParamGroups.find(name);
		if( it == ParamGroups.end() )
		{	//无此参数退出
			xSemaphoreGive(ParamGroup_Semphr);
			return PR_ERR;
		}
		group = it->second;
	
		xSemaphoreGive(ParamGroup_Semphr);
	}
	
	ParamGroupInf* group_inf = (ParamGroupInf*)group->data;
	if( group_inf->version & (1<<14) )
	{	//不为普通参数组退出
		return PR_ERR;
	}
	
	//判断参数类别
	bool is_param1 = ( group_inf->version & (1<<15) ) != 0;
	uint16_t read_to = start + read_count;
	if( start>=group_inf->params_count || read_to>group_inf->params_count )
		return PR_ERR;
	
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake(group->ReadSemphr, TIMEOUT_Ticks) == pdTRUE )
	{
		if( is_new != 0 )
			*is_new = group->is_new;
		for( uint16_t i = start ; i < read_to ; ++i )
		{
			if( is_param1 )
			{
				ParamData1* param = (ParamData1*)&((uint8_t*)group->data)[sizeof(ParamGroupInf)+i*sizeof(ParamData1)];
				data[i-start] = param->data;
			}
			else
			{
				ParamData2* param = (ParamData2*)&((uint8_t*)group->data)[sizeof(ParamGroupInf)+i*sizeof(ParamData2)];
				data[i-start] = param->data;
			}
		}
		xSemaphoreGive(group->ReadSemphr);
		return PR_OK;
	}
	return PR_TimeOut;
}

/*
	读取指定名称参数组参数个数
	注意：仅可在参数表初始化完成后调用
	name：参数组名称
	count：读取的参数组长度
	TIMEOUT：超时时间

	返回值：
	PR_ERR：参数表中无此参数或读取的参数超出参数范围
	PR_TimeOut：访问参数表操作超时
	PR_OK：成功
*/
PR_RESULT GetVolatileParamGroupParamCount( SName name, uint16_t* count, double TIMEOUT )
{
	//参数表未完成初始化不允许更新参数
	if( getInitializationCompleted() == false )
		return PR_ERR;
	
	//获取参数组
	VolatileParamGroup* group;
	if( xSemaphoreTake( ParamGroup_Semphr, portMAX_DELAY ) == pdTRUE )
	{
		map<SName, ParamGroup*>::iterator it = ParamGroups.find(name);
		if( it == ParamGroups.end() )
		{	//无此参数退出
			xSemaphoreGive(ParamGroup_Semphr);
			return PR_ERR;
		}
		group = (VolatileParamGroup*)it->second;
	
		xSemaphoreGive(ParamGroup_Semphr);
	}
	
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	
	if( xSemaphoreTake( group->ReadSemphr, TIMEOUT_Ticks ) == pdTRUE )
	{
		*count = group->params_count;
		
		xSemaphoreGive(group->ReadSemphr);
		return PR_OK;
	}
	else
		return PR_TimeOut;
}

/*
	读取指定名称参数组
	注意：仅可在参数表初始化完成后调用
	name：参数组名称
	data：要读取的参数组参数的值
	start：从序号start开始更新（0开始）
	read_count：读取数目
	TIMEOUT：超时时间

	返回值：
	PR_ERR：参数表中无此参数或读取的参数超出参数范围
	PR_TimeOut：访问参数表操作超时
	PR_OK：成功
*/
PR_RESULT ReadVolatileParamGroup( SName name, void* data, uint16_t start, uint16_t read_count, double TIMEOUT )
{
	//参数表未完成初始化不允许更新参数
	if( getInitializationCompleted() == false )
		return PR_ERR;
	
	if( read_count==0 )
		return PR_ERR;
	
	//获取参数组
	VolatileParamGroup* group;
	if( xSemaphoreTake( ParamGroup_Semphr, portMAX_DELAY ) == pdTRUE )
	{
		map<SName, ParamGroup*>::iterator it = ParamGroups.find(name);
		if( it == ParamGroups.end() )
		{	//无此参数退出
			xSemaphoreGive(ParamGroup_Semphr);
			return PR_ERR;
		}
		group = (VolatileParamGroup*)it->second;
	
		xSemaphoreGive(ParamGroup_Semphr);
	}
	
	VolatileParamGroupInf* group_inf = (VolatileParamGroupInf*)group->data;
	if( (group_inf->version >> 14) != 1 )
	{	//不为变长参数组退出
		return PR_ERR;
	}
	
	//参数名
	char name_ch[17];
	name.get_CharStr(name_ch);
	
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	
	if( xSemaphoreTake( group->ReadSemphr, TIMEOUT_Ticks ) == pdTRUE )
	{
		uint32_t read_to = (uint32_t)start + (uint32_t)read_count;
		if( read_to > group->params_count )
		{	//参数超出范围退出
			xSemaphoreGive(group->ReadSemphr);
			return PR_ERR;
		}
		
		//判断是否缓存参数
		if( group->ram_param_count >= read_count )
		{	//可缓存参数
			SS_RESULT res;
			if( group->ram_param_start>start || group->ram_param_start+group->ram_param_count<read_to )
			{	//需要更新参数范围
			
				//如果内存中的参数未同步到存储器
				if( group->ram_updated && group->params_count>group->ram_param_start )
				{	//先把参数更新到存储器
					uint32_t save_param_count = group->params_count - group->ram_param_start;
					if( group->ram_param_count < save_param_count )
						save_param_count = group->ram_param_count;
					if( group->is_new )
					{
						SS_RESULT res = InternalStorage_WriteFile( "Config", name_ch, group->data, 
									0,
									sizeof(VolatileParamGroupInf));					
						if( res != SS_OK )
						{					
							xSemaphoreGive(group->ReadSemphr);
							return PR_ERR;
						}
						group->is_new = false;
					}
					res = InternalStorage_WriteFile( "Config", name_ch, &((uint8_t*)group->data)[sizeof(VolatileParamGroupInf)],
								sizeof(VolatileParamGroupInf) + group_inf->param_length*group->ram_param_start,
								group_inf->param_length*save_param_count );
				}
				
				//求读取存储器的参数个数
				uint32_t rd_param_count = group->params_count - start;
				if( group->ram_param_count < rd_param_count )
						rd_param_count = group->ram_param_count;
				uint8_t* rdData = new uint8_t[rd_param_count*group_inf->param_length];		
				
				//读取存储器
				uint32_t rd_param_bytes;
				res = InternalStorage_ReadFile( "Config", name_ch, rdData, &rd_param_bytes, 
							sizeof(VolatileParamGroupInf) + group_inf->param_length*start,
							group_inf->param_length*rd_param_count );
					
				//更新内存参数
				if( res==SS_OK )
				{
					//更新内存参数范围
					group->ram_param_start = start;
					group->ram_updated = false;
					//将读取的参数复制进内存
					memcpy( 
						&((uint8_t*)group->data)[sizeof(VolatileParamGroupInf)] ,
						rdData ,
						rd_param_count*group_inf->param_length );
						
					//复制参数给用户
					memcpy( 
						data ,
						rdData ,
						read_count*group_inf->param_length );
						
					delete[] rdData;
				}
				else
				{
					xSemaphoreGive(group->ReadSemphr);
					delete[] rdData;
					return PR_TimeOut;
				}
			}
			else
			{	//无需更新参数范围
				
				//复制参数给用户
				memcpy( 
					data ,
					&((uint8_t*)group->data)[sizeof(VolatileParamGroupInf)+group_inf->param_length*(start-group->ram_param_start)] ,
					read_count*group_inf->param_length );
			}
		}
		else
		{	//不缓存参数

			//如果内存中的参数未同步到存储器
			if( group->ram_updated && group->params_count>group->ram_param_start )
			{
				uint32_t save_param_count = group->params_count - group->ram_param_start;
				if( group->ram_param_count < save_param_count )
					save_param_count = group->ram_param_count;
				if( group->is_new )
				{
					SS_RESULT res = InternalStorage_WriteFile( "Config", name_ch, group->data, 
								0,
								sizeof(VolatileParamGroupInf));					
					if( res != SS_OK )
					{					
						xSemaphoreGive(group->ReadSemphr);
						return PR_ERR;
					}
					group->is_new = false;
				}
				SS_RESULT res = InternalStorage_WriteFile( "Config", name_ch, &((uint8_t*)group->data)[sizeof(VolatileParamGroupInf)],
													sizeof(VolatileParamGroupInf) + group_inf->param_length*group->ram_param_start,
													group_inf->param_length*save_param_count );
				if( res != SS_OK )
				{
					xSemaphoreGive(group->ReadSemphr);
					return PR_ERR;
				}
				group->ram_updated = false;
			}
			
			//存储器读取参数
			uint32_t rd_count = 0;
			SS_RESULT res = InternalStorage_ReadFile( "Config", name_ch, data, &rd_count, 
												sizeof(VolatileParamGroupInf) + group_inf->param_length*start,
												group_inf->param_length*read_count );
			if( res != SS_OK )
			{
				xSemaphoreGive(group->ReadSemphr);
				return PR_ERR;
			}
		}
		xSemaphoreGive(group->ReadSemphr);
		return PR_OK;
	}
	else
		return PR_TimeOut;
}

/*
	保存指定名称可变参数组
	注意：仅可在参数表初始化完成后调用
	name：参数组名称
	TIMEOUT：超时时间

	返回值：
	PR_ERR：参数表中无此参数或读取的参数超出参数范围
	PR_TimeOut：访问参数表操作超时
	PR_OK：成功
*/
PR_RESULT SaveVolatileParamGroup( SName name, double TIMEOUT )
{
	//参数表未完成初始化不允许更新参数
	if( getInitializationCompleted() == false )
		return PR_ERR;
	
	//获取参数组
	VolatileParamGroup* group;
	if( xSemaphoreTake( ParamGroup_Semphr, portMAX_DELAY ) == pdTRUE )
	{
		map<SName, ParamGroup*>::iterator it = ParamGroups.find(name);
		if( it == ParamGroups.end() )
		{	//无此参数退出
			xSemaphoreGive(ParamGroup_Semphr);
			return PR_ERR;
		}
		group = (VolatileParamGroup*)it->second;
	
		xSemaphoreGive(ParamGroup_Semphr);
	}
	
	VolatileParamGroupInf* group_inf = (VolatileParamGroupInf*)group->data;
	if( (group_inf->version >> 14) != 1 )
	{	//不为变长参数组退出
		return PR_ERR;
	}
	
	//参数名
	char name_ch[17];
	name.get_CharStr(name_ch);
	
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake( group->ReadSemphr, TIMEOUT_Ticks ) == pdTRUE )
	{
		//如果内存中的参数未同步到存储器
		if( group->ram_updated && group->params_count>group->ram_param_start )
		{	//先把参数更新到存储器
			uint32_t save_param_count = group->params_count - group->ram_param_start;
			if( group->ram_param_count < save_param_count )
				save_param_count = group->ram_param_count;
			if( group->is_new )
			{
				SS_RESULT res = InternalStorage_WriteFile( "Config", name_ch, group->data, 
							0,
							sizeof(VolatileParamGroupInf));					
				if( res != SS_OK )
				{					
					xSemaphoreGive(group->ReadSemphr);
					return PR_ERR;
				}
				group->is_new = false;
			}
			SS_RESULT res = InternalStorage_WriteFile( "Config", name_ch, &((uint8_t*)group->data)[sizeof(VolatileParamGroupInf)], 
												sizeof(VolatileParamGroupInf) + group_inf->param_length*group->ram_param_start,
												group_inf->param_length*save_param_count );
			if( res != SS_OK )
			{
				xSemaphoreGive(group->ReadSemphr);
				return PR_ERR;
			}
			group->ram_updated = false;
		}
					
		xSemaphoreGive(group->ReadSemphr);
		return PR_OK;
	}
	else
		return PR_TimeOut;
}

/*
	保存指定名称可变参数组
	注意：仅可在参数表初始化完成后调用
	name：参数组名称
	TIMEOUT：超时时间

	返回值：
	PR_ERR：参数表中无此参数或读取的参数超出参数范围
	PR_TimeOut：访问参数表操作超时
	PR_OK：成功
*/
PR_RESULT TruncateVolatileParamGroup( SName name, uint16_t size, double TIMEOUT )
{
	//参数表未完成初始化不允许更新参数
	if( getInitializationCompleted() == false )
		return PR_ERR;
	
	//获取参数组
	VolatileParamGroup* group;
	if( xSemaphoreTake( ParamGroup_Semphr, portMAX_DELAY ) == pdTRUE )
	{
		map<SName, ParamGroup*>::iterator it = ParamGroups.find(name);
		if( it == ParamGroups.end() )
		{	//无此参数退出
			xSemaphoreGive(ParamGroup_Semphr);
			return PR_ERR;
		}
		group = (VolatileParamGroup*)it->second;
	
		xSemaphoreGive(ParamGroup_Semphr);
	}
	
	VolatileParamGroupInf* group_inf = (VolatileParamGroupInf*)group->data;
	if( (group_inf->version >> 14) != 1 )
	{	//不为变长参数组退出
		return PR_ERR;
	}
	
	//参数名
	char name_ch[17];
	name.get_CharStr(name_ch);
	
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake( group->ReadSemphr, TIMEOUT_Ticks ) == pdTRUE )
	{
		if( group->params_count < size )
		{
			xSemaphoreGive(group->ReadSemphr);
			return PR_ERR;
		}
		
		SS_RESULT res = InternalStorage_TruncateFile( "Config", name_ch, 
											sizeof(VolatileParamGroupInf) + group_inf->param_length*size );
		if( res != SS_OK )
		{			
			xSemaphoreGive(group->ReadSemphr);
			return PR_ERR;
		}
		group->params_count = size;
					
		xSemaphoreGive(group->ReadSemphr);
		return PR_OK;
	}
	else
		return PR_TimeOut;
}

/*
	更新指定名称可变参数组
	注意：仅可在参数表初始化完成后调用
	name：参数组名称
	data：要更新的参数组参数的值
	start：从序号start开始更新（0开始）
	write_count：更新数目
	st：是否写入存储器
	TIMEOUT：超时时间

	返回值：
	PR_ERR：参数表中无此参数或读取的参数超出参数范围
	PR_TimeOut：访问参数表操作超时
	PR_OK：成功
*/
PR_RESULT WriteVolatileParamGroup( SName name, const void* data, uint16_t start, uint16_t write_count, bool st, double TIMEOUT )
{
	//参数表未完成初始化不允许更新参数
	if( getInitializationCompleted() == false )
		return PR_ERR;
	
	//数据太多退出
	uint32_t write_to = (uint32_t)start + (uint32_t)write_count;
	if( write_to>MAX_VOLATILE_PARAMS || write_count==0 )
		return PR_ERR;
	
	//获取参数组
	VolatileParamGroup* group;
	if( xSemaphoreTake( ParamGroup_Semphr, portMAX_DELAY ) == pdTRUE )
	{
		map<SName, ParamGroup*>::iterator it = ParamGroups.find(name);
		if( it == ParamGroups.end() )
		{	//无此参数退出
			xSemaphoreGive(ParamGroup_Semphr);
			return PR_ERR;
		}
		group = (VolatileParamGroup*)it->second;
	
		xSemaphoreGive(ParamGroup_Semphr);
	}
	
	VolatileParamGroupInf* group_inf = (VolatileParamGroupInf*)group->data;
	if( (group_inf->version >> 14) != 1 )
	{	//不为变长参数组退出
		return PR_ERR;
	}
	
	//参数名
	char name_ch[17];
	name.get_CharStr(name_ch);
	
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake( group->ReadSemphr, TIMEOUT_Ticks ) == pdTRUE )
	{
		if( start > group->params_count )
		{	//参数超出范围退出
			xSemaphoreGive(group->ReadSemphr);
			return PR_ERR;
		}
		
		//判断是否缓存参数
		if( st==false && group->ram_param_count>=write_count )
		{	//可缓存参数
			SS_RESULT res;
			if( group->ram_param_start>start || group->ram_param_start+group->ram_param_count<write_to )
			{	//需要更新参数范围
			
				//如果内存中的参数未同步到存储器
				if( group->ram_updated && group->params_count>group->ram_param_start )
				{	//先把参数更新到存储器
					uint32_t save_param_count = group->params_count - group->ram_param_start;
					if( group->ram_param_count < save_param_count )
						save_param_count = group->ram_param_count;
					if( group->is_new )
					{
						SS_RESULT res = InternalStorage_WriteFile( "Config", name_ch, group->data, 
									0,
									sizeof(VolatileParamGroupInf));					
						if( res != SS_OK )
						{					
							xSemaphoreGive(group->ReadSemphr);
							return PR_ERR;
						}
						group->is_new = false;
					}
					res = InternalStorage_WriteFile( "Config", name_ch, &((uint8_t*)group->data)[sizeof(VolatileParamGroupInf)],
								sizeof(VolatileParamGroupInf) + group_inf->param_length*group->ram_param_start,
								group_inf->param_length*save_param_count );
				}
				
				//求读取存储器的参数个数
				uint32_t rd_param_count = group->params_count - start;
				if( group->ram_param_count < rd_param_count )
						rd_param_count = group->ram_param_count;
				if( rd_param_count > write_count )
					rd_param_count -= write_count;
				else
					rd_param_count = 0;
				uint8_t* rdData = new uint8_t[rd_param_count*group_inf->param_length];			
				
				uint32_t rd_param_bytes = 0;
				if( rd_param_count > 0 )
					res = InternalStorage_ReadFile( "Config", name_ch, rdData, &rd_param_bytes, 
								sizeof(VolatileParamGroupInf) + group_inf->param_length*(start+write_count),
								group_inf->param_length*rd_param_count );
				else
					res = SS_OK;
					
				//更新内存参数
				if( res==SS_OK )
				{
					//更新内存参数范围
					group->ram_param_start = start;
					if( start + write_count > group->params_count )
						group->params_count = start + write_count;
					//将新写入的参数写到内存
					memcpy( 
						&((uint8_t*)group->data)[sizeof(VolatileParamGroupInf)] ,
						(uint8_t*)data ,
						write_count*group_inf->param_length );
					//将读取的参数复制进内存
					memcpy( 
						&((uint8_t*)group->data)[sizeof(VolatileParamGroupInf)+group_inf->param_length*write_count] ,
						rdData ,
						rd_param_count*group_inf->param_length );
					group->ram_updated = true;
					delete[] rdData;
				}
				else
				{
					xSemaphoreGive(group->ReadSemphr);
					delete[] rdData;
					return PR_ERR;
				}
			}
			else
			{	//无需更新参数范围
					
				//更新内存参数范围
				if( start + write_count > group->params_count )
					group->params_count = start + write_count;
				group->ram_updated = true;
				//将新写入的参数写到内存
				memcpy( 
					&((uint8_t*)group->data)[sizeof(VolatileParamGroupInf)+group_inf->param_length*(start-group->ram_param_start)] ,
					(uint8_t*)data ,
					write_count*group_inf->param_length );
			}
		}
		else
		{	//不缓存参数
			
			//求内存参数更新范围
			uint32_t ram_param_end = group->ram_param_start+group->ram_param_count;
			if( ram_param_end > MAX_VOLATILE_PARAMS )
				ram_param_end = MAX_VOLATILE_PARAMS;
			uint32_t ram_write_start = 
				( group->ram_param_start > start ) ? group->ram_param_start : start;			
			uint32_t ram_write_to = 
				( ram_param_end < write_to ) ? ram_param_end : write_to;
			
			//更新内存参数范围
			if( start + write_count > group->params_count )
				group->params_count = start + write_count;
			if( ram_write_start < ram_write_to )
			{
				//将新写入的参数写到内存
				memcpy( 
					&((uint8_t*)group->data)[sizeof(VolatileParamGroupInf)+group_inf->param_length*(ram_write_start-group->ram_param_start)] ,
					&((uint8_t*)data)[group_inf->param_length*(ram_write_start-start)] ,
					(ram_write_to-ram_write_start)*group_inf->param_length );					
			}
			
			//新参数写入存储器
			InternalStorage_WriteFile( "Config", name_ch, data, 
				sizeof(VolatileParamGroupInf) + group_inf->param_length*start,
				group_inf->param_length*write_count );
		}
		
		xSemaphoreGive(group->ReadSemphr);
		return PR_OK;
	}
	else
		return PR_TimeOut;
}

/*
	更新指定名称参数组
	注意：仅可在参数表初始化完成后调用
	name：参数组名称
	data：要更新的参数组参数的值
	start：从序号start开始更新（0开始）
	write_count：更新数目
	st：是否写入存储器
	TIMEOUT：超时时间

	返回值：
	PR_ERR：参数表中无此参数或读取的参数超出参数范围
	PR_TimeOut：访问参数表操作超时
	PR_OK：成功
*/
PR_RESULT UpdateParamGroup( SName name, const uint64_t data[], uint16_t start, uint16_t write_count, bool st, double TIMEOUT )
{
	//参数表未完成初始化不允许更新参数
	if( getInitializationCompleted() == false )
		return PR_ERR;
	
	//获取参数组
	ParamGroup* group;
	if( xSemaphoreTake( ParamGroup_Semphr, portMAX_DELAY ) == pdTRUE )
	{
		map<SName, ParamGroup*>::iterator it = ParamGroups.find(name);
		if( it == ParamGroups.end() )
		{	//无此参数退出
			xSemaphoreGive(ParamGroup_Semphr);
			return PR_ERR;
		}
		group = it->second;
	
		xSemaphoreGive(ParamGroup_Semphr);
	}
	
	ParamGroupInf* group_inf = (ParamGroupInf*)group->data;
	uint16_t write_to = start + write_count;
	if( start>=group_inf->params_count || write_to>group_inf->params_count )
	{	//参数超出范围退出
		return PR_ERR;
	}
	
	if( group_inf->version & (1<<14) )
	{	//不为普通参数组退出
		return PR_ERR;
	}
	
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake( Params_Semphr, TIMEOUT_Ticks ) == pdTRUE )
	{		
		bool is_param1 = ( group_inf->version & (1<<15) ) != 0;
		//在内存中更新参数
		if( xSemaphoreTake(group->ReadSemphr, TIMEOUT_Ticks) == pdTRUE )
		{
			for( uint16_t i = start ; i < write_to ; ++i )
			{
				if( is_param1 )
				{
					ParamData1* param = (ParamData1*)&((uint8_t*)group->data)[sizeof(ParamGroupInf)+i*sizeof(ParamData1)];
					param->data = data[i-start];
				}
				else
				{
					ParamData2* param = (ParamData2*)&((uint8_t*)group->data)[sizeof(ParamGroupInf)+i*sizeof(ParamData2)];
					param->data = data[i-start];
				}
			}
			xSemaphoreGive(group->ReadSemphr);
		}
		else
		{
			xSemaphoreGive(Params_Semphr);
			return PR_TimeOut;
		}
		
		if(st)
		{	//保存参数到存储器
			char name_ch[17];
			name.get_CharStr(name_ch);
			uint32_t ParamGroup_length;
			if( is_param1 )
				ParamGroup_length = sizeof(ParamGroupInf) + sizeof(ParamData1)*group_inf->params_count;
			else
				ParamGroup_length = sizeof(ParamGroupInf) + sizeof(ParamData2)*group_inf->params_count;
			SS_RESULT ss_res = InternalStorage_SaveFile( "Config", name_ch, group->data, ParamGroup_length );
			if( ss_res == SS_OK )
				group->is_new = false;
			xSemaphoreGive(Params_Semphr);
			if( ss_res == SS_OK )
				return PR_OK;
			else if( ss_res == SS_TimeOut )
				return PR_TimeOut;
			else
				return PR_ERR;
		}
		xSemaphoreGive(Params_Semphr);
		return PR_OK;
	}
	else
		return PR_TimeOut;
}

/*
	将指定名称参数组名称的参数保存到存储器
	注意：仅可在参数表初始化完成后调用
	name：参数组名称
	TIMEOUT：超时时间

	返回值：
	PR_ERR：参数表中无此参数或读取的参数超出参数范围
	PR_TimeOut：访问参数表操作超时
	PR_OK：保存成功
*/
PR_RESULT SaveParamGroup( SName name, double TIMEOUT )
{
	//参数表未完成初始化不允许更新参数
	if( getInitializationCompleted() == false )
		return PR_ERR;
	
	//获取参数组
	ParamGroup* group;
	if( xSemaphoreTake( ParamGroup_Semphr, portMAX_DELAY ) == pdTRUE )
	{
		map<SName, ParamGroup*>::iterator it = ParamGroups.find(name);
		if( it == ParamGroups.end() )
		{	//无此参数退出
			xSemaphoreGive(ParamGroup_Semphr);
			return PR_ERR;
		}
		group = it->second;
	
		xSemaphoreGive(ParamGroup_Semphr);
	}
	
	ParamGroupInf* group_inf = (ParamGroupInf*)group->data;
	if( group_inf->version & (1<<14) )
	{	//不为普通参数组退出
		return PR_ERR;
	}
	
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake( Params_Semphr, TIMEOUT_Ticks ) == pdTRUE )
	{		
		bool is_param1 = ( group_inf->version & (1<<15) ) != 0;
		
		//保存参数到存储器
		char name_ch[17];
		name.get_CharStr(name_ch);
		uint32_t ParamGroup_length;
		if( is_param1 )
			ParamGroup_length = sizeof(ParamGroupInf) + sizeof(ParamData1)*group_inf->params_count;
		else
			ParamGroup_length = sizeof(ParamGroupInf) + sizeof(ParamData2)*group_inf->params_count;
		SS_RESULT ss_res = InternalStorage_SaveFile( "Config", name_ch, group->data, ParamGroup_length );
		if( ss_res == SS_OK )
			group->is_new = false;
		xSemaphoreGive(Params_Semphr);
		if( ss_res == SS_OK )
			return PR_OK;
		else if( ss_res == SS_TimeOut )
			return PR_TimeOut;
		else
			return PR_ERR;
	}
	else
		return PR_TimeOut;
}

/*
	更新指定名称参数
	注意：仅可在参数表初始化完成后调用
	name：参数组名称
	data：要更新的参数组参数的值
	TIMEOUT：超时时间

	返回值：
	PR_ERR：参数表中无此参数
	PR_TimeOut：访问参数表操作超时
	PR_OK：更新成功
*/
PR_RESULT UpdateParam( SName name, const uint64_t data, double TIMEOUT )
{
	//参数表未完成初始化不允许更新参数
	if( getInitializationCompleted() == false )
		return PR_ERR;
	
	map<SName, Param>::iterator it = GroupParameters.find(name);
	if( it == GroupParameters.end() )
		//无此参数退出
		return PR_ERR;
	Param param = it->second;
	ParamGroup* group = param.group;
	
	TickType_t TIMEOUT_Ticks;
	if( TIMEOUT >= 0 )
		TIMEOUT_Ticks = TIMEOUT*configTICK_RATE_HZ;
	else
		TIMEOUT_Ticks = portMAX_DELAY;
	if( xSemaphoreTake( Params_Semphr, TIMEOUT_Ticks ) == pdTRUE )
	{
		//在内存中更新参数
		if( xSemaphoreTake(group->ReadSemphr, TIMEOUT_Ticks) == pdTRUE )
		{
			param.data->data = data;
			xSemaphoreGive(group->ReadSemphr);
		}
		else
		{
			xSemaphoreGive(Params_Semphr);
			return PR_TimeOut;
		}
		//保存参数到存储器
		char name_ch[17];
		group->name.get_CharStr(name_ch);
		ParamGroupInf* group_inf = (ParamGroupInf*)group->data;
		bool is_param1 = ( group_inf->version & (1<<15) ) != 0;
		uint32_t ParamGroup_length;
		if( is_param1 )
			ParamGroup_length = sizeof(ParamGroupInf) + sizeof(ParamData1)*group_inf->params_count;
		else
			ParamGroup_length = sizeof(ParamGroupInf) + sizeof(ParamData2)*group_inf->params_count;
		SS_RESULT ss_res = InternalStorage_SaveFile( "Config", name_ch, group->data, ParamGroup_length );
		if( ss_res == SS_OK )
			group->is_new = false;
		xSemaphoreGive(Params_Semphr);
		if( ss_res == SS_OK )
			return PR_OK;
		else if( ss_res == SS_TimeOut )
			return PR_TimeOut;
		else
			return PR_ERR;
	}
	else
		return PR_TimeOut;
}


void init_Parameters()
{
	ParamGroup_Semphr = xSemaphoreCreateMutex();
	Params_Semphr = xSemaphoreCreateMutex();
}