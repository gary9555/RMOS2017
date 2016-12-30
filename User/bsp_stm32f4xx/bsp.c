/*
*********************************************************************************************************
*
*	模块名称 : BSP模块(For STM32F4XX)
*	文件名称 : bsp.c
*	版    本 : V1.1
*	说    明 : 这是硬件底层驱动程序的主文件。每个c文件可以 #include "bsp.h" 来包含所有的外设驱动模块。
*			   bsp = Borad surport packet 板级支持包
*
*********************************************************************************************************
*/
#include "includes.h"

void NVIC_Configuration(void);

/* 
********************************************************************************************************* 
* 																						REGISTERS 
********************************************************************************************************* 
*/
#define BSP_REG_DEM_CR 						(*(CPU_REG32 *)0xE000EDFC) 
#define BSP_REG_DWT_CR 						(*(CPU_REG32 *)0xE0001000) 
#define BSP_REG_DWT_CYCCNT 				(*(CPU_REG32 *)0xE0001004) 
#define BSP_REG_DBGMCU_CR 				(*(CPU_REG32 *)0xE0042004)

/* 
********************************************************************************************************* 
* 																						REGISTER BITS 
********************************************************************************************************* 
*/
#define BSP_DBGMCU_CR_TRACE_IOEN_MASK 0x10 
#define BSP_DBGMCU_CR_TRACE_MODE_ASYNC 0x00 
#define BSP_DBGMCU_CR_TRACE_MODE_SYNC_01 0x40 
#define BSP_DBGMCU_CR_TRACE_MODE_SYNC_02 0x80 
#define BSP_DBGMCU_CR_TRACE_MODE_SYNC_04 0xC0 
#define BSP_DBGMCU_CR_TRACE_MODE_MASK 0xC0

#define BSP_BIT_DEM_CR_TRCENA DEF_BIT_24 
#define BSP_BIT_DWT_CR_CYCCNTENA DEF_BIT_00


/*
*********************************************************************************************************
*	函 数 名: bsp_Init
*	功能说明: 初始化所有的硬件设备。该函数配置CPU寄存器和外设的寄存器并初始化一些全局变量。只需要调用一次
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_Init(void)
{
	/*
		由于ST固件库的启动文件已经执行了CPU系统时钟的初始化，所以不必再次重复配置系统时钟。
		启动文件配置了CPU主时钟频率、内部Flash访问速度和可选的外部SRAM FSMC初始化。

		系统时钟缺省配置为168MHz，如果需要更改，可以修改 system_stm32f4xx.c 文件
	*/
	NVIC_Configuration();	
	bsp_InitUart(); 	/* init uart */

	bsp_InitLed(); 		// init led port
	//bsp_InitKey();
	
#ifdef TRACE_EN
	BSP_CPU_REG_DBGMCU_CR |= BSP_DBGMCU_CR_TRACE_IOEN_MASK; // Enable tracing
	BSP_CPU_REG_DBGMCU_CR &= ~BSP_DBGMCU_CR_TRACE_MODE_MASK; // Clr trace mode sel bits.
	BSP_CPU_REG_DBGMCU_CR |= BSP_DBGMCU_CR_TRACE_MODE_SYNC_04; // Cfg trace mode to synch 4-bit.
#endif
	
}

/* 
********************************************************************************************************* 
* 		 NVIC_Configuration 
********************************************************************************************************* 
*/
void NVIC_Configuration(void){
	// 设置NVIC优先级分组为group2：0-3 抢占式优先级，0-3的响应式优先级
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); 
}

/* 
********************************************************************************************************* 
* BSP_CPU_ClkFreq() 
* Description : Read CPU registers to determine the CPU clock frequency of the chip. 
* Argument(s) : none. 
* Return(s) : The CPU clock frequency, in Hz. 
* Caller(s) : Application. 
* Note(s) : none. 
********************************************************************************************************* 
*/
CPU_INT32U BSP_CPU_ClkFreq (void){
	RCC_ClocksTypeDef rcc_clocks; 
	RCC_GetClocksFreq(&rcc_clocks); 
	return ((CPU_INT32U)rcc_clocks.HCLK_Frequency); 
}


/* 
********************************************************************************************************* 
* BSP_Tick_Init() 
* Description : Initialize all the peripherals that required OS Tick services (OS initialized) 
* Argument(s) : none. 
* Return(s) : none. 
* Caller(s) : Application. 
* Note(s) : none. 
********************************************************************************************************* 
*/
void BSP_Tick_Init (void){
	
	CPU_INT32U cpu_clk_freq; 
	CPU_INT32U cnts; 
	cpu_clk_freq = BSP_CPU_ClkFreq(); // Determine SysTick reference freq. 
	
#if (OS_VERSION >= 30000u) 
	cnts = cpu_clk_freq / (CPU_INT32U)OSCfg_TickRate_Hz; // Determine nbr SysTick increments.  
#else 
	cnts = cpu_clk_freq / (CPU_INT32U)OS_TICKS_PER_SEC; // Determine nbr SysTick increments. 
#endif 
	
	OS_CPU_SysTickInit(cnts);  // 这里默认的是最高优先级
	
	// SysTick_Config(cnts);   // 这里默认的是最低优先级
}

/* ********************************************************************************************************* 
* CPU_TS_TmrInit() 
* 
* Description : Initialize & start CPU timestamp timer. 
* 
* Argument(s) : none. 
* 
* Return(s) : none. 
* 
* Caller(s) : CPU_TS_Init(). 
* 
* This function is an INTERNAL CPU module function & MUST be implemented by application/ 
* BSP function(s) [see Note #1] but MUST NOT be called by application function(s). 
* 
* Note(s) : (1) CPU_TS_TmrInit() is an application/BSP function that MUST be defined by the developer 
* if either of the following CPU features is enabled : 
* 
* (a) CPU timestamps 
* (b) CPU interrupts disabled time measurements 
* 
* See 'cpu_cfg.h CPU TIMESTAMP CONFIGURATION Note #1' 
* & 'cpu_cfg.h CPU INTERRUPTS DISABLED TIME MEASUREMENT CONFIGURATION Note #1a'. 
* 
* (2) (a) Timer count values MUST be returned via word-size-configurable 'CPU_TS_TMR' 
* data type. 
* 
* (1) If timer has more bits, truncate timer values' higher-order bits greater 
* than the configured 'CPU_TS_TMR' timestamp timer data type word size. 
*
* (2) Since the timer MUST NOT have less bits than the configured 'CPU_TS_TMR' 
* timestamp timer data type word size; 'CPU_CFG_TS_TMR_SIZE' MUST be 
* configured so that ALL bits in 'CPU_TS_TMR' data type are significant. 
* 
* In other words, if timer size is not a binary-multiple of 8-bit octets 
* (e.g. 20-bits or even 24-bits), then the next lower, binary-multiple 
* octet word size SHOULD be configured (e.g. to 16-bits). However, the 
* minimum supported word size for CPU timestamp timers is 8-bits. 
* 
* See also 'cpu_cfg.h CPU TIMESTAMP CONFIGURATION Note #2' 
* & 'cpu_core.h CPU TIMESTAMP DATA TYPES Note #1'. 
* 
* (b) Timer SHOULD be an 'up' counter whose values increase with each time count. 
* 
* (c) When applicable, timer period SHOULD be less than the typical measured time 
* but MUST be less than the maximum measured time; otherwise, timer resolution 
* inadequate to measure desired times. 
* 
* See also 'CPU_TS_TmrRd() Note #2'. 
********************************************************************************************************* 
*/
#if (CPU_CFG_TS_TMR_EN == DEF_ENABLED) 
void CPU_TS_TmrInit(void){ 
	CPU_INT32U fclk_freq; 
	fclk_freq = BSP_CPU_ClkFreq(); 
	BSP_REG_DEM_CR |= (CPU_INT32U)BSP_BIT_DEM_CR_TRCENA; // Enable Cortex-M4's DWT CYCCNT reg. 
	BSP_REG_DWT_CYCCNT = (CPU_INT32U)0u; 
	BSP_REG_DWT_CR |= (CPU_INT32U)BSP_BIT_DWT_CR_CYCCNTENA; 
	CPU_TS_TmrFreqSet((CPU_TS_TMR_FREQ)fclk_freq); 
} 
#endif

/* 
********************************************************************************************************* 
* CPU_TS_TmrRd() 
* 
* Description : Get current CPU timestamp timer count value. 
* 
* Argument(s) : none. 
* 
* Return(s) : Timestamp timer count (see Notes #2a & #2b). 
* 
* Caller(s) : CPU_TS_Init(), 
* CPU_TS_Get32(), 
* CPU_TS_Get64(), 
* CPU_IntDisMeasStart(), 
* CPU_IntDisMeasStop(). 
* 
* This function is an INTERNAL CPU module function & MUST be implemented by application/ 
* BSP function(s) [see Note #1] but SHOULD NOT be called by application function(s). 
*
* Note(s) : (1) CPU_TS_TmrRd() is an application/BSP function that MUST be defined by the developer 
* if either of the following CPU features is enabled : 
* 
* (a) CPU timestamps 
* (b) CPU interrupts disabled time measurements 
* 
* See 'cpu_cfg.h CPU TIMESTAMP CONFIGURATION Note #1' 
* & 'cpu_cfg.h CPU INTERRUPTS DISABLED TIME MEASUREMENT CONFIGURATION Note #1a'. 
* 
* (2) (a) Timer count values MUST be returned via word-size-configurable 'CPU_TS_TMR' 
* data type. 
* 
* (1) If timer has more bits, truncate timer values' higher-order bits greater 
* than the configured 'CPU_TS_TMR' timestamp timer data type word size. 
* 
* (2) Since the timer MUST NOT have less bits than the configured 'CPU_TS_TMR' 
* timestamp timer data type word size; 'CPU_CFG_TS_TMR_SIZE' MUST be 
* configured so that ALL bits in 'CPU_TS_TMR' data type are significant. 
* 
* In other words, if timer size is not a binary-multiple of 8-bit octets 
* (e.g. 20-bits or even 24-bits), then the next lower, binary-multiple 
* octet word size SHOULD be configured (e.g. to 16-bits). However, the 
* minimum supported word size for CPU timestamp timers is 8-bits. 
* 
* See also 'cpu_cfg.h CPU TIMESTAMP CONFIGURATION Note #2' 
* & 'cpu_core.h CPU TIMESTAMP DATA TYPES Note #1'. 
* 
* (b) Timer SHOULD be an 'up' counter whose values increase with each time count. 
* 
* (1) If timer is a 'down' counter whose values decrease with each time count, 
* then the returned timer value MUST be ones-complemented. 
* 
* (c) (1) When applicable, the amount of time measured by CPU timestamps is 
* calculated by either of the following equations : 
* 
* (A) Time measured = Number timer counts * Timer period 
* 
* where 
* 
* Number timer counts Number of timer counts measured 
* Timer period Timer's period in some units of 
* (fractional) seconds 
* Time measured Amount of time measured, in same 
* units of (fractional) seconds 
* as the Timer period 
* 
* 											Number timer counts 
* (B) Time measured = --------------------- 
* 												Timer frequency 
* 
* where 
* 
* Number timer counts Number of timer counts measured 
* Timer frequency Timer's frequency in some units 
* of counts per second 
* Time measured Amount of time measured, in seconds 
* 
* (2) Timer period SHOULD be less than the typical measured time but MUST be less
* than the maximum measured time; otherwise, timer resolution inadequate to 
* measure desired times. 
********************************************************************************************************* 
*/
#if (CPU_CFG_TS_TMR_EN == DEF_ENABLED) 
CPU_TS_TMR CPU_TS_TmrRd (void){
	CPU_TS_TMR ts_tmr_cnts; 
	ts_tmr_cnts = (CPU_TS_TMR)BSP_REG_DWT_CYCCNT; 
	return (ts_tmr_cnts); 
} 
#endif


/* ********************************************************************************************************* 
* CPU_TSxx_to_uSec() 
* 
* Description : Convert a 32-/64-bit CPU timestamp from timer counts to microseconds. 
* 
* Argument(s) : ts_cnts CPU timestamp (in timestamp timer counts [see Note #2aA]). 
* 
* Return(s) : Converted CPU timestamp (in microseconds [see Note #2aD]). 
* 
* Caller(s) : Application. 
* 
* This function is an (optional) CPU module application programming interface (API) 
* function which MAY be implemented by application/BSP function(s) [see Note #1] & 
* MAY be called by application function(s). 
* 
* Note(s) : (1) CPU_TS32_to_uSec()/CPU_TS64_to_uSec() are application/BSP functions that MAY be 
* optionally defined by the developer when either of the following CPU features is 
* enabled : 
* 
* (a) CPU timestamps 
* (b) CPU interrupts disabled time measurements 
* 
* See 'cpu_cfg.h CPU TIMESTAMP CONFIGURATION Note #1' 
* & 'cpu_cfg.h CPU INTERRUPTS DISABLED TIME MEASUREMENT CONFIGURATION Note #1a'. 
* 
* (2) (a) The amount of time measured by CPU timestamps is calculated by either of 
* the following equations : 
* 
* 																						10^6 microseconds 
* (1) Time measured = Number timer counts * ------------------- * Timer period 
* 																								1 second 
* 
* 											Number timer counts 		10^6 microseconds 
* (2) Time measured = --------------------- * ------------------- 
* 												Timer frequency 					1 second 
* 
* where 
* 
* (A) Number timer counts Number of timer counts measured 
* (B) Timer frequency Timer's frequency in some units
* of counts per second 
* (C) Timer period Timer's period in some units of 
* (fractional) seconds 
* (D) Time measured Amount of time measured, 
* in microseconds 
* 
* (b) Timer period SHOULD be less than the typical measured time but MUST be less 
* than the maximum measured time; otherwise, timer resolution inadequate to 
* measure desired times. 
* 
* (c) Specific implementations may convert any number of CPU_TS32 or CPU_TS64 bits 
* -- up to 32 or 64, respectively -- into microseconds. 
********************************************************************************************************* 
*/
#if (CPU_CFG_TS_32_EN == DEF_ENABLED) 
CPU_INT64U CPU_TS32_to_uSec (CPU_TS32 ts_cnts){ 
	CPU_INT64U ts_us; 
	CPU_INT64U fclk_freq; 
	fclk_freq = BSP_CPU_ClkFreq(); 
	ts_us = ts_cnts / (fclk_freq / DEF_TIME_NBR_uS_PER_SEC); 
	return (ts_us); } 
#endif 
	
#if (CPU_CFG_TS_64_EN == DEF_ENABLED) 
CPU_INT64U CPU_TS64_to_uSec(CPU_TS64 ts_cnts){ 
	CPU_INT64U ts_us; 
	CPU_INT64U fclk_freq; 
	fclk_freq = BSP_CPU_ClkFreq(); 
	ts_us = ts_cnts / (fclk_freq / DEF_TIME_NBR_uS_PER_SEC); 
	return (ts_us); 
} 
#endif


/*
*********************************************************************************************************
*	函 数 名: bsp_RunPer10ms
*	功能说明: 该函数每隔10ms被Systick中断调用1次。详见 bsp_timer.c的定时中断服务程序。一些需要周期性处理
*			的事务可以放在此函数。比如：按键扫描、蜂鸣器鸣叫控制等。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_RunPer10ms(void)
{

}

/*
*********************************************************************************************************
*	函 数 名: bsp_RunPer1ms
*	功能说明: 该函数每隔1ms被Systick中断调用1次。详见 bsp_timer.c的定时中断服务程序。一些需要周期性处理的
*			事务可以放在此函数。比如：触摸坐标扫描。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_RunPer1ms(void)
{
	//TOUCH_Scan();	/* 触摸扫描 */
}

/*
*********************************************************************************************************
*	函 数 名: bsp_Idle
*	功能说明: 空闲时执行的函数。一般主程序在for和while循环程序体中需要插入 CPU_IDLE() 宏来调用本函数。
*			 本函数缺省为空操作。用户可以添加喂狗、设置CPU进入休眠模式的功能。
*	形    参：无
*	返 回 值: 无
*********************************************************************************************************
*/
void bsp_Idle(void)
{
	/* --- 喂狗 */

	/* --- 让CPU进入休眠，由Systick定时中断唤醒或者其他中断唤醒 */

	/* 对于 emWin 图形库，可以插入图形库需要的轮询函数 */
	//GUI_Exec();

	/* 对于 uIP 协议实现，可以插入uip轮询函数 */
}

/*
*********************************************************************************************************
*    name: bsp_DelayUS()
*    function: CPU internal counter,32bit 
*                OSSchedLock(&err);
*                bsp_DelayUS(5);
*                OSSchedUnlock(&err); ??????????????????????
*    parameter: _ulDelayTime  delay length 1 us
*    retval: void
*    specification: 1. when main freq = 168MHz,32bit counter at most 2^32/168000000 = 25.565sec
*                
*             2. ?????????,????????????????0.25us??????
*             ????????:
*             (1). MDK5.15,optimize level 0, independent of different MDK optimization level 
*             (2). STM32F407IGT6
*             (3). test method:
*                 GPIOI->BSRRL = GPIO_Pin_8;
*                 bsp_DelayUS(10);
*                 GPIOI->BSRRH = GPIO_Pin_8;
*             -------------------------------------------
*                test                 actual time lapse
*             bsp_DelayUS(1)          1.2360us
*             bsp_DelayUS(2)          2.256us
*             bsp_DelayUS(3)          3.256us
*             bsp_DelayUS(4)          4.256us
*             bsp_DelayUS(5)          5.276us
*             bsp_DelayUS(6)          6.276us
*             bsp_DelayUS(7)          7.276us
*             bsp_DelayUS(8)          8.276us
*             bsp_DelayUS(9)          9.276us
*             bsp_DelayUS(10)         10.28us
*            3. 2 32-bit unsigned number deduction, the result assigned to 32-bit can still 
								render the correct difference
*              if A,B,C are all 32-bit unsigned numbers
*              if A > B  then A - B = C,
*              if A < B  then A - B = C, C shall have the value of 
*						   0xFFFFFFFF - B + A + 1. 这点要注意，正好用于本函数
*********************************************************************************************************
*/
void bsp_DelayUS(uint32_t _ulDelayTime){
    uint32_t tCnt, tDelayCnt;
    uint32_t tStart;

    tStart = (uint32_t)CPU_TS_TmrRd();                       /* tick value while entering */
    tCnt = 0;
    tDelayCnt = _ulDelayTime * (SystemCoreClock / 1000000);     /* ticks needed */               

    while(tCnt < tDelayCnt)
    {
        tCnt = (uint32_t)CPU_TS_TmrRd() - tStart; /* ?????,???????32????????,???????? */    
    }
}
