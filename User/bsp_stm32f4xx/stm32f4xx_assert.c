/*
*********************************************************************************************************
*	                                  
*	ģ������ : ����ģ�顣
*	�ļ����� : stm32f4xx_assert.c
*	��    �� : V1.0
*	˵    �� : �ṩ���Ժ���,��Ҫ���ڳ�����ԡ�ST�̼����еĺ��������Զ�����������м�飬��߳���Ľ�׳�ԡ�
*			   ����ļ��������ڱ�׼����ļ���ST�̼���ķ�������Щ��������main.c�ļ���
*			   ������Ϊ������û��ľ���Ӧ���޹أ���˽������������ʹmain.c�ļ����������Ӽ��һЩ��
*
*********************************************************************************************************
*/

#include "stm32f4xx.h"	
#include <stdio.h>

/* 
	ST�⺯��ʹ����C�������Ķ��Թ��ܣ����������USE_FULL_ASSERT����ô���е�ST�⺯������麯���β�
	�Ƿ���ȷ���������ȷ������ assert_failed() ���������������һ����ѭ���������û������롣
	
	�ؼ��� __LINE__ ��ʾԴ�����кš�
	�ؼ���__FILE__��ʾԴ�����ļ�����
	
	���Թ���ʹ�ܺ���������С���Ƽ��û����ڵ���ʱʹ�ܣ�����ʽ���������ǽ�ֹ��

	�û�����ѡ���Ƿ�ʹ��ST�̼���Ķ��Թ��ܡ�ʹ�ܶ��Եķ��������֣�
	(1) ��C��������Ԥ�����ѡ���ж���USE_FULL_ASSERT��
	(2) �ڱ��ļ�ȡ��"#define USE_FULL_ASSERT    1"�е�ע�͡�	
*/
#ifdef USE_FULL_ASSERT

/*
*********************************************************************************************************
*	�� �� ��: assert_failed
*	��    �Σ�file : Դ�����ļ����ơ��ؼ���__FILE__��ʾԴ�����ļ�����
*			  line �������кš��ؼ��� __LINE__ ��ʾԴ�����к�
*	�� �� ֵ: ��
*********************************************************************************************************
*/
void assert_failed(uint8_t* file, uint32_t line)
{ 
	/* 
		�û����������Լ��Ĵ��뱨��Դ�����ļ����ʹ����кţ����罫�����ļ����кŴ�ӡ������
		printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	*/
	
	/* ����һ����ѭ��������ʧ��ʱ������ڴ˴��������Ա����û���� */
	while (1)
	{
	}
}
#endif