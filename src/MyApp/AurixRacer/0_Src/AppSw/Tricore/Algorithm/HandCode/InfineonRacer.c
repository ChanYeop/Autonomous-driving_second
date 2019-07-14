/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/
#include "InfineonRacer.h"
#include "Basic.h"
#define ABS(x) ( ((x)<0)?-(x):(x) ) //���밪

/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*--------------------------------Enumerations--------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*------------------------------Global variables------------------------------*/
/******************************************************************************/
InfineonRacer_t IR_Ctrl  /**< \brief  global data */
      = {LEFT_MARGIN, RIGHT_MARGIN, FALSE};
const float32 dt = 0.02;
const float32 EMA_a = 0.4; //PID ���� ���� ��ó���� low pass filter

PID_t PID_dc =
{
		.target = 40,
		.current_val = 0,
		.error = 0,
		.pre_error = 0,
		.Kp = 0.2,
		.Ki = 0.5,
		.Kd = 0,
		.i_error = 0,
		.d_error = 0,
		.p_term = 0,
		.i_term = 0,
		.d_term = 0,
		.adjust = 0
};

PID_t PID_srv =
{
		.target = LEFT_MARGIN,
		.current_val = 0,
		.error = 0,
		.pre_error = 0,
		.Kp = 0.02,
		.Ki = 0,
		.Kd = 0,
		.i_error = 0,
		.d_error = 0,
		.p_term = 0,
		.i_term = 0,
		.d_term = 0,
		.adjust = 0
};

float32 diffArr[DIFF_ARR_SIZE]; //adcResult�� �̺��� �� DIFF_THRESHOLD �ȳѴ� ���� 0���� ����� �����ϴ� array
float32 filteredArr[FILTERED_ARR_SIZE]; //diffArr�� ��� ���ͷ� ���͸��� �� �����ϴ� array
float32 twoDiffArr[TWO_DIFF_ARR_SIZE]; //filteredArr�� �̺��� �� �����ϴ� array
boolean conArr[TWO_DIFF_ARR_SIZE]; //twoDiffArr�� �� threshold(LINE_THRESHOLD)�� �̻��� ���� 1 �ƴϸ� 0(consider array)

uint32 right_line = 128; //right_line�� ���� index ����
uint32 left_line = 127; //left_line�� ���� index ����

uint32 left = 0;
uint32 right = 1; //left camera or right camera

boolean school_zone = FALSE; //���������� �ƴ��� �Ǻ�
boolean crosswalk_ing = FALSE; //Ⱦ�ܺ��� ������ �ƴ��� �Ǻ�

boolean check_line_left = FALSE; //���� ���� ���� ����
boolean check_line_right = FALSE; //������ ���� ���� ����

uint32 space_count = 0; //���� ������ ���� count
boolean left2 = FALSE; //2�������� ���� ���� ǥ��
boolean right2 = FALSE; //2�������� ������ ���� ǥ��

uint32 school_zone_timer = 0;
uint32 turn_timer = 0;

boolean turning = FALSE;
boolean left_turn_flag = FALSE;
boolean right_turn_flag = FALSE;

uint32 total_line[256] = {};
uint32 final_line[256] = {};
uint32 raw_line[256] = {};

uint32 dist_flag;
float32 distance = 0;

const float32 speed_zone_velocity = 100;
const float32 school_zone_velocity = 50;

/******************************************************************************/
/*-------------------------Function Prototypes--------------------------------*/
/******************************************************************************/
void makeDiff(int lr); //window size�� �̺��Ͽ� �Ӱ谪(DIFF_THRESHOLD)���� clipping�� �� diffArr�� ����
void averageFilter(void); //filter window size�� ���͸��� �� filteredArr�� ����
void makeTwoDiff(void); //window size�� ���� �̺��� �� twoDiffArr�� ����
void considerLine(void); //twoDiffArr���� line���� �ǽɵǴ� index���� 1���� �ƴϸ� 0

void right_index_number(void); //�߾����κ��� ���������� ù��° line index�� right_line�� ����
void left_index_number(void); //�߾����κ��� ���� ù��° line index�� left_line�� ����

void countLine(void); //line���� count�Ͽ� IR_LineScan.lineCount�� ����

void check_crosswalk(void); // "school zone" or "racing zone" �Ǵ�

void final_right_line(void); // �������� �߽ɿ� ���� ����� ������ index�� right_line ������ ����
void final_left_line(void); // �������� �߽ɿ� ���� ����� ������ index�� left_line ������ ����

void PID_control_dc(void);
void PID_control_srv(void);
/******************************************************************************/
/*------------------------Private Variables/Constants-------------------------*/
/******************************************************************************/

/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/

//window size�� �̺��� �� diffArr�� ����
//+ �Ӱ谪(DIFF_THRESHOLD) �ȳѴ� �� 0���� ����
//lr : left or right
void makeDiff(int lr)
{
   for(int i = 0; i < DIFF_ARR_SIZE; i++)
   {
      diffArr[i] = ((float32)IR_LineScan.adcResult[lr][i + (WINDOW_SIZE - 1)] - (float32)IR_LineScan.adcResult[lr][i]) / WINDOW_SIZE;
      diffArr[i] = (ABS(diffArr[i]) < DIFF_THRESHOLD) ? 0 : diffArr[i];
   }
}

//��� ����. filter window size�� ���͸��� �� filteredArr�� ����
void averageFilter(void)
{
   int sum;
   for(int i = 0; i < FILTERED_ARR_SIZE; i++)
   {
      sum = 0;
      for(int j = 0; j < FILTER_SIZE; j++)
      {
         sum += diffArr[i + j];
      }
      filteredArr[i] = (float32)sum / FILTER_SIZE;
   }
}

//window size�� ���� �̺��� �� twoDiffArr�� ����
void makeTwoDiff(void)
{
   for(int i = 0; i < TWO_DIFF_ARR_SIZE; i++)
   {
      twoDiffArr[i] = (filteredArr[i + (WINDOW_SIZE - 1)] - filteredArr[i]) / WINDOW_SIZE;
   }
}

//twoDiffArr���� line���� �ǽɵǴ� index���� 1���� �ƴϸ� 0
void considerLine(void)
{
   for(int i = 0; i < TWO_DIFF_ARR_SIZE; i++)
   {
      if(twoDiffArr[i] > LINE_THRESHOLD)
         conArr[i] = TRUE;
      else
         conArr[i] = FALSE;
   }
}

void right_index_number(void)
{
   right_line = LS_SIZE;
   check_line_right = TRUE;

   for(int i = 0; i < LS_SIZE; i++)
   {
      if(IR_LineScan.line[right][i] == 4095)
      {
         right_line = i + LS_SIZE;
         break;
      }

      if(i == LS_SIZE - 1)
      {
    	  right_line = 2 * LS_SIZE - 1;
    	  check_line_right = FALSE; //������ ���� ���� ���ϸ� false
      }
   }
}

void left_index_number(void)
{
   left_line = 127;
   check_line_left = TRUE;

   for(int i = LS_SIZE-1; i >= 0; i--)
   {
      if(IR_LineScan.line[left][i] == 4095)
      {
         left_line = i;
         break;
      }

      if(i == 0)
      {
    	  left_line = 0;
    	  check_line_left = FALSE; //���� ���� ���� ���ϸ� false
      }
   }
}

//line���� count�Ͽ� IR_LineScan.lineCount�� ����
void countLine(void)
{
	uint32 tempCount = 0;
	boolean countFlag = FALSE;
	for(int i = 0; i < LS_SIZE * 2; i++)
	{
		if(((i < 128) ? IR_LineScan.line[left][i] : IR_LineScan.line[right][i - 128]) == 4095)
		{
			if(!countFlag)
			{
				tempCount++;
				countFlag = TRUE;
			}
		}
		else
		{
			countFlag = FALSE;
		}
	}

	IR_LineScan.lineCount = tempCount;
}

void final_right_line(void)
{
   makeDiff(right);
   averageFilter();
   makeTwoDiff();

   considerLine();

   for(int i = 0; i < (WINDOW_SIZE - 1) + FILTER_SIZE / 2; i++)
   {
      IR_LineScan.line[right][i] = 0;
   }
   for(int i = (WINDOW_SIZE - 1) + FILTER_SIZE / 2; i < TWO_DIFF_ARR_SIZE + (WINDOW_SIZE - 1) + FILTER_SIZE / 2; i++)
   {
      IR_LineScan.line[right][i] = (conArr[i - ((WINDOW_SIZE - 1) + (FILTER_SIZE - 1) / 2)] == TRUE) ? 4095 : 0;
   }
   for(int i = TWO_DIFF_ARR_SIZE + (WINDOW_SIZE - 1) + FILTER_SIZE / 2; i < LS_SIZE; i++)
   {
      IR_LineScan.line[right][i] = 0;
   }
   right_index_number();
}

void final_left_line(void)
{
   makeDiff(left);
   averageFilter();
   makeTwoDiff();
   considerLine();

   for(int i = 0; i < (WINDOW_SIZE - 1) + FILTER_SIZE / 2; i++)
   {
      IR_LineScan.line[left][i] = 0;
   }
   for(int i = (WINDOW_SIZE - 1) + FILTER_SIZE / 2; i < TWO_DIFF_ARR_SIZE + (WINDOW_SIZE - 1) + FILTER_SIZE / 2; i++)
   {
      IR_LineScan.line[left][i] = (conArr[i - ((WINDOW_SIZE - 1) + (FILTER_SIZE - 1) / 2)] == TRUE) ? 4095 : 0;
   }
   for(int i = TWO_DIFF_ARR_SIZE + (WINDOW_SIZE - 1) + FILTER_SIZE / 2; i < LS_SIZE; i++)
   {
      IR_LineScan.line[left][i] = 0;
   }
   left_index_number();

}

void check_crosswalk(void)
{
   if(IR_LineScan.lineCount > LINE_COUNT_THRESHOLD && crosswalk_ing == FALSE)
   {
	   if(school_zone == FALSE)
	   {
		   if(school_zone_timer > 100)
		   { //school_zone�� FALSE�� �Ǹ� �ٷ� TRUE�� �ٷ� �ٲ�� �� ����
		   		school_zone = TRUE;
		   		school_zone_timer = 0;
		   }
	   }
	   else
	   {
		   if(school_zone_timer > 100)
		   { //school_zone�� TRUE�� �Ǹ� �ٷ� FALSE�� �ٷ� �ٲ�� �� ����
			   school_zone = FALSE;
			   school_zone_timer = 0;
		   }
	   }
	   crosswalk_ing = TRUE;
   }
   else if(IR_LineScan.lineCount > LINE_COUNT_THRESHOLD)
	   crosswalk_ing = TRUE;
   else
	   crosswalk_ing = FALSE;
}

void PID_control_dc(void)
{
	//���� ���: 0.3, 0.5
	PID_dc.current_val = IR_getEncSpeed();
	PID_dc.error = PID_dc.target - PID_dc.current_val;

	PID_dc.p_term = PID_dc.Kp * PID_dc.error;

	PID_dc.i_error += PID_dc.error * dt;
	PID_dc.i_term = PID_dc.Ki * PID_dc.i_error;

	//PID_dc.d_error = (PID_dc.error * PID_dc.pre_error) / dt;
	//PID_dc.d_term = PID_dc.Kd * PID_dc.d_error;

	PID_dc.pre_error = PID_dc.error;

	//PID_dc.adjust = (PID_dc.p_term + PID_dc.i_term + PID_dc.d_term) / 100.0;
	PID_dc.adjust = (PID_dc.p_term + PID_dc.i_term ) / 100.0;

	if(PID_dc.adjust > 1.0)
		PID_dc.adjust = 1.0;
	if(PID_dc.adjust < -1.0)
		PID_dc.adjust = -1.0;

	IR_setMotor0Vol(PID_dc.adjust);
}

void PID_control_srv(void)
{
	if(school_zone == FALSE) // school_zone �ܺο��� ���� �۵� ���
	{
		if(check_line_left && check_line_right) // ���ʰ� ������ ���� �� �� �νĽ� �̿�
		{
			PID_srv.target = 0;
			PID_srv.current_val = (float32)(2 * LS_SIZE) - (float32)left_line - (float32)right_line;
			PID_srv.error = PID_srv.target - PID_srv.current_val;
		}
		else if(check_line_left)
		{
			PID_srv.target = IR_Ctrl.Ls0Margin;
			PID_srv.current_val = (float32)LS_SIZE - (float32)left_line; //���� ls0 margin ���
			PID_srv.error =  PID_srv.target - PID_srv.current_val;
		}
		else
		{ //���� ������ �Ⱥ��̴� ��Ȳ���� ������ ���� �̿�
			PID_srv.target = IR_Ctrl.Ls1Margin;
			PID_srv.current_val = (float32)right_line - (float32)LS_SIZE; //���� ls1 margin ���
			PID_srv.error = PID_srv.current_val - PID_srv.target;
		}
	}
	else // school_zone ���ο��� ���� �۵� ���
	{
		if(turning == FALSE) // ������
		{
			if(left2 == FALSE && right2 == FALSE)
			{
				if(check_line_left)
				{
					PID_srv.target = IR_Ctrl.Ls0Margin;
					PID_srv.current_val = LS_SIZE - left_line; //���� ls0 margin ���
					PID_srv.error =  PID_srv.target - PID_srv.current_val;
				}
				else
				{
					PID_srv.target = IR_Ctrl.Ls1Margin;
					PID_srv.current_val = right_line - LS_SIZE; //���� ls1 margin ���
					PID_srv.error = PID_srv.current_val - PID_srv.target;
				}
			}
			else if(left2 == TRUE) // ���� ���� �϶�, ������ ����, ���� ���� �̿�
			{
				PID_srv.target = IR_Ctrl.Ls0Margin;
				PID_srv.current_val = LS_SIZE - left_line; //���� ls0 margin ���
				PID_srv.error =  PID_srv.target - PID_srv.current_val;
			}
			else if(right2 == TRUE) // ������ ���� �϶�, ���� ����, ������ ���� �̿�
			{
				PID_srv.target = IR_Ctrl.Ls1Margin;
				PID_srv.current_val = right_line - LS_SIZE; //���� ls1 margin ���
				PID_srv.error = PID_srv.current_val - PID_srv.target;
			}
		}
///////////////////////////////////////////////////////////////////////////////////////
		else // ȸ����
		{
			if(right_turn_flag) // ���������� ȸ��
			{
				IR_setSrvAngle(0.4);
			}
			else if(left_turn_flag)// �������� ȸ��
			{
				IR_setSrvAngle(-0.4);
			}
		}
////////////////////////////////////////////////////////////////////////////////////////
	}

	if(turning == FALSE) // ������, ȸ�����϶��� ���� ���ε�
	{
		PID_srv.p_term = PID_srv.Kp * PID_srv.error;

		PID_srv.i_error += PID_srv.error * dt;
		PID_srv.i_term = PID_srv.Ki * PID_srv.i_error;

		PID_srv.d_error = (PID_srv.error * PID_srv.pre_error) / dt;
		PID_srv.d_term = PID_srv.Kd * PID_srv.d_error;

		PID_srv.pre_error = PID_srv.error;

		PID_srv.adjust = EMA_a * (PID_srv.p_term + PID_srv.i_term + PID_srv.d_term) + (1 - EMA_a) * PID_srv.adjust; //low pass filter ������ control ��

		if(PID_srv.adjust > 0.4)
			PID_srv.adjust = 0.4;
		if(PID_srv.adjust < -0.4)
			PID_srv.adjust = -0.4;

		IR_setSrvAngle(PID_srv.adjust);
	}
}

void LED_test(void)
{
	(school_zone == TRUE) ? IR_setLed0(1) : IR_setLed0(0); // DIG 33 pin

	if(school_zone == FALSE)
	{
    	(check_line_left == TRUE) ? IR_setLed1(1) : IR_setLed1(0); // DIG 31 pin
       (check_line_right == TRUE) ? IR_setLed2(1) : IR_setLed2(0); // DIG 29 pin
	}
	else
	{
    	(left2 == TRUE) ? IR_setLed1(1) : IR_setLed1(0); // DIG 31 pin
       (right2 == TRUE) ? IR_setLed2(1) : IR_setLed2(0); // DIG 29 pin
	}
	(IR_Distance[0] < 80) ? IR_setLed3(1) : IR_setLed3(0); // DIG 34 pin
	(IR_Distance[1] < 80) ? IR_setLed4(1) : IR_setLed4(0); // DIG 35 pin
}

void InfineonRacer_init(void)
{

}

void speed_reduction (void)//������ �ۿ��� ����Ǿ�� ��.
{
	distance = (IR_Distance[0] + IR_Distance[1]) / 2;

	if(school_zone == FALSE)//������ �ۿ����� ����Ǵ� �ڵ�
	{
		//���ܼ� ���� �Ÿ��� ���� �ӵ�����
		if(distance > 80)
		{
			dist_flag = 3;//nothing
		}
		else if(distance > 60)
		{
			dist_flag = 2;//far
		}
		else if(distance > 40)
		{
			dist_flag = 1;//mid
		}
		else if(distance > 15)
		{
			dist_flag = 0;//close
		}

		if(dist_flag == 3)//nothing
		{
			PID_dc.target = speed_zone_velocity;//������ �ӵ������� ����. ���� 100
		}
		else if(dist_flag == 2)//far
		{
			PID_dc.target = speed_zone_velocity / 1.5;//����1�ܰ�
		}
		else if(dist_flag == 1)//mid
		{
			PID_dc.target = speed_zone_velocity / 2;//����2�ܰ�
		}
		else if(dist_flag == 0)//close
		{
			IR_setMotor0Vol(0);
			PID_dc.Kp = 0;
			PID_dc.Ki = 0;
		}

		//���������� ���� �ӵ�����
		//IR_Srv.Angle�� -0.4~ 0.4 �� ���� ����.
		if(dist_flag == 3)
		{
			if((IR_Srv.Angle >= 0.3) || (IR_Srv.Angle < -0.3))//���� ���� ������
			{
				PID_dc.target = speed_zone_velocity / 2;
			}
			else if((IR_Srv.Angle > 0.2) || (IR_Srv.Angle < -0.2))
			{
				PID_dc.target = speed_zone_velocity / 1.5;
			}
			else
			{
				PID_dc.target = speed_zone_velocity;
			}
		}
	}
}

void obstacle_detect(void) // ��ֹ� ����
{
	if(school_zone)
	{
		if(IR_Distance[0] < 80 || IR_Distance[1] < 80)
		{
			turning = TRUE;
		}

		if(turning == TRUE)
		{
			if(left2 == TRUE) //���� �������� ���� �־����� ���������� ȸ��
			{
				right_turn_flag = TRUE;
				left2 = FALSE;
				right2 = TRUE;
			}
			else //������ �������� ���� �־����� �������� ȸ��
			{
				left_turn_flag = TRUE;
				left2 = TRUE;
				right2 = FALSE;
			}

			if(turn_timer > TURN_TIME)
			{
				left_turn_flag = FALSE;
				right_turn_flag = FALSE;
				turning = FALSE;
				turn_timer = 0;
			}
		}
	}
	else//������ �ƴѵ� ��ֹ� ���� �Ǿ�����
	{
		speed_reduction();
	}
}

void InfineonRacer_detectLane(void)
{
   final_left_line(); // �������� �߽ɿ� ���� ����� ������ index�� left_line ������ ����
   final_right_line(); // �������� �߽ɿ� ���� ����� ������ index�� right_line ������ ����

   for(int i = LS_SIZE - 1 - 15; i >= 0; i--)
   {
	   IR_LineScan.line[0][i + 15] = IR_LineScan.line[0][i];
   }
   for(int i = 0; i < (LS_SIZE - 15); i++)
   {
	   IR_LineScan.line[1][i] = IR_LineScan.line[1][i + 15];
   }
   for(int i = 0; i < 15; i++)
   {
	   IR_LineScan.line[0][i] = 0;
	   IR_LineScan.line[1][LS_SIZE - 1 - i] = 0;
   }

   countLine();
   check_crosswalk();
   for(int i = 0; i < 128; i++)
   {
	   total_line[i] = IR_LineScan.line[0][i];
	   raw_line[i] = IR_LineScan.adcResult[0][i];
   }
   for(int i = 128; i < 256; i++)
   {
	   total_line[i] = IR_LineScan.line[1][i - 128];
	   raw_line[i] = IR_LineScan.adcResult[1][i - 128];
   }

   if(school_zone)
   {
	   if(IR_LineScan.lineCount == 1) //���� ������ �������� linecount�� 1
		   space_count++;
	   else
		   space_count = 0;

	   if(left2 == FALSE && right2 == FALSE)
	   {
		   if(space_count >= 2)
		   {
			   if(check_line_left == TRUE && check_line_right == FALSE)
			   {
				   left2 = TRUE; // ���� ������ �ִ�.
				   right2 = FALSE; // ������ ������ ����
			   }
			   else if(check_line_left == FALSE && check_line_right == TRUE)
			   {
				   right2 = TRUE; // ������ ������ �ִ�.
				   left2 = FALSE; // ���� ������ ����
				   //������ ������ �����̸� ���� ������ ���� ����. ���� ������ �����̸� ������ ������ ���� ����.
				   //���� �����ϱ� ��(left2, right2 �� �� FALSE)���� 1����ó�� �� �� ���� ����
				   //(�ϳ��ϳ� �� �������� 1���� -> 2�������� �������� �������� ���� �߻��� ������ �ֱ� �ѵ� ���� ������ ��. �����ϱ�δ� ���� ���� ��)
			   }
		   }
	   }
   }
}

void InfineonRacer_control(void){
	if(school_zone)
	{
		PID_dc.target = school_zone_velocity;
	}
	else
	{
		PID_dc.target = speed_zone_velocity;
	}
	obstacle_detect();

	PID_control_dc();
	PID_control_srv();
	LED_test();
}
