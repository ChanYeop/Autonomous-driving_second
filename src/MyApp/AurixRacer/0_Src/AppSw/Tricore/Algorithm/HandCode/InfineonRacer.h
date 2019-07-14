#ifndef INFINEONRACER_H_
#define INFINEONRACER_H_


/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include <Ifx_Types.h>
#include "Configuration.h"

/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/
#define IR_getLs0Margin()		IR_Ctrl.Ls0Margin
#define IR_getLs1Margin()		IR_Ctrl.Ls1Margin

#define LS_SIZE 128 //line scan ī�޶� ��ü ũ��(1�� ���� 128, 2�� ���� 256)
#define WINDOW_SIZE 5 //�̺��ϴ� window�� ũ��
#define FILTER_SIZE 9 //��� ���� window�� ũ��

#define DIFF_ARR_SIZE 		LS_SIZE - (WINDOW_SIZE - 1) //�̺е� array ũ�� : ī�޶� ��ü ũ�� - (window size - 1)
#define FILTERED_ARR_SIZE 	DIFF_ARR_SIZE - (FILTER_SIZE - 1) //���͸��� array ũ�� : �̺� array ũ�� - (filter window size - 1)
#define TWO_DIFF_ARR_SIZE 	FILTERED_ARR_SIZE - (WINDOW_SIZE - 1) //���� �̺е� array ũ�� : ���͸��� array ũ�� - (window size - 1)

#define DIFF_THRESHOLD 100
#define LINE_THRESHOLD 10
#define LINE_COUNT_THRESHOLD 6  //line ������ LINE_COUNT_THRESHOLD���� Ŭ �� Ⱦ�ܺ��� �Ǻ��뵵

#define LEFT_MARGIN 90
#define RIGHT_MARGIN 90

#define TURN_TIME 100 //turn �ϴ� �ð�. 10ms����
/******************************************************************************/
/*--------------------------------Enumerations--------------------------------*/
/******************************************************************************/



/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/
typedef struct{
	sint32 Ls0Margin;
	sint32 Ls1Margin;
	boolean basicTest;
}InfineonRacer_t;

typedef struct{
	float32 target;
	float32 current_val;

	float32 error;
	float32 pre_error;

	float32 i_error;
	float32 d_error;

	float32 Kp;
	float32 Ki;
	float32 Kd;

	float32 p_term;
	float32 i_term;
	float32 d_term;

	float32 adjust;
}PID_t;

/******************************************************************************/
/*------------------------------Global variables------------------------------*/
/******************************************************************************/
IFX_EXTERN InfineonRacer_t IR_Ctrl;
IFX_EXTERN PID_t PID_dc;
IFX_EXTERN PID_t PID_srv;

IFX_EXTERN uint32 school_zone_timer;
IFX_EXTERN boolean school_zone;

IFX_EXTERN boolean turning;
IFX_EXTERN uint32 turn_timer;
/******************************************************************************/
/*-------------------------Function Prototypes--------------------------------*/
/******************************************************************************/
IFX_EXTERN void InfineonRacer_init(void);
IFX_EXTERN void InfineonRacer_detectLane(void);
IFX_EXTERN void InfineonRacer_control(void);

#endif
