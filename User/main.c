#include "includes.h"

/* 
********************************************************************************************************* 
* 																	LOCAL GLOBAL VARIABLES 
********************************************************************************************************* 
*/
static OS_TCB AppTaskStartTCB; 
static CPU_STK AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE]; 
//static OS_TCB AppTaskUpdateTCB; 
//static CPU_STK AppTaskUpdateStk[APP_CFG_TASK_UPDATE_STK_SIZE]; 
//static OS_TCB AppTaskCOMTCB; 
//static CPU_STK AppTaskCOMStk[APP_CFG_TASK_COM_STK_SIZE]; 
static OS_TCB AppTaskUserIFTCB; 
static CPU_STK AppTaskUserIFStk[APP_CFG_TASK_USER_IF_STK_SIZE];


/* 
********************************************************************************************************* 
* 																		FUNCTION PROTOTYPES 
********************************************************************************************************* 
*/
static void AppTaskCreate (void); 
static void AppTaskStart (void *p_arg); 
static void AppTaskUserIF (void *p_arg); 
//static void AppTaskCOM (void *p_arg);


int main(void){
	
	OS_ERR err; 
	// BSP_IntDisAll(); // Disable all interrupts. 
	
	
	OSInit(&err); // Init uC/OS-III. 
	
	OSTaskCreate((OS_TCB *)&AppTaskStartTCB, // Create the start task  
							 (CPU_CHAR *)"App Task Start", 
							 (OS_TASK_PTR )AppTaskStart, 
							 (void *)0, 
							 (OS_PRIO )APP_CFG_TASK_START_PRIO, 
							 (CPU_STK *)&AppTaskStartStk[0], 
							 (CPU_STK_SIZE )APP_CFG_TASK_START_STK_SIZE / 10, 
							 (CPU_STK_SIZE )APP_CFG_TASK_START_STK_SIZE, 
							 (OS_MSG_QTY )0, 
							 (OS_TICK )0, 
							 (void *)0, 
							 (OS_OPT )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR), 
							 (OS_ERR *)&err); 
							 
	OSStart(&err); // Start multitasking (i.e. give control to uC/OS-III).  
  (void)&err; 
	return (0);
	
}

/* 
********************************************************************************************************* 
* 	AppTaskStart: This is an init task
* 	retval: void
* 	priority:2 
********************************************************************************************************* 
*/
static void AppTaskStart (void *p_arg){ 
	OS_ERR err; 
	(void)p_arg; 
	bsp_Init(); 
	CPU_Init(); 
	BSP_Tick_Init(); 
	// Mem_Init(); 
	// Math_Init(); 
#if OS_CFG_STAT_TASK_EN > 0u 
	OSStatTaskCPUUsageInit(&err); 
#endif 
	
#ifdef CPU_CFG_INT_DIS_MEAS_EN 
	CPU_IntDisMeasMaxCurReset(); 
#endif 
	
	AppTaskCreate(); 	
	while(1){ 
		bsp_LedToggle(1);
		OSTimeDly(100, OS_OPT_TIME_DLY, &err);
	}
}

static void AppTaskUserIF(void *p_arg){ 
	OS_ERR err; 
	while(1){ 
		bsp_LedToggle(4); 
		OSTimeDly(100, OS_OPT_TIME_DLY, &err); 
	} 
}

/* 
********************************************************************************************************* 
* 	AppTaskCreate 
* 	retval: void
* 	p_arg 
********************************************************************************************************* 
*/
static void AppTaskCreate (void) { 
	OS_ERR err;
	
	OSTaskCreate((OS_TCB *)&AppTaskUserIFTCB,
							 (CPU_CHAR *)"App Task UserIF", 
							 (OS_TASK_PTR )AppTaskUserIF, 
							 (void *)0, 
							 (OS_PRIO )APP_CFG_TASK_USER_IF_PRIO, 
							 (CPU_STK *)&AppTaskUserIFStk[0], 
							 (CPU_STK_SIZE )APP_CFG_TASK_USER_IF_STK_SIZE / 10, 
							 (CPU_STK_SIZE )APP_CFG_TASK_USER_IF_STK_SIZE, 
							 (OS_MSG_QTY )0, 
							 (OS_TICK )0, 
							 (void *)0, 
							 (OS_OPT )(OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR), 
							 (OS_ERR *)&err);
	
}

