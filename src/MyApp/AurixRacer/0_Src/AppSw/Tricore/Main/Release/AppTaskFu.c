#include "AppTaskFu.h"

static sint32 task_cnt_1m = 0;
static sint32 task_cnt_10m = 0;
static sint32 task_cnt_100m = 0;
static sint32 task_cnt_1000m = 0;

boolean task_flag_1m = FALSE;
boolean task_flag_10m = FALSE;
boolean task_flag_100m = FALSE;
boolean task_flag_1000m = FALSE;

void appTaskfu_init(void){
	BasicLineScan_init();
	BasicPort_init();
    BasicGtmTom_init();
    BasicVadcBgScan_init();
    BasicGpt12Enc_init();
    AsclinShellInterface_init();

#if BOARD == APPLICATION_KIT_TC237
    tft_app_init(1);
    perf_meas_init();
#elif BOARD == SHIELD_BUDDY

#endif

#if CODE == CODE_HAND
    InfineonRacer_init();
#elif CODE == CODE_ERT
    IR_Controller_initialize();
#else

#endif
}

void appTaskfu_1ms(void)
{
	task_cnt_1m++;
	if(task_cnt_1m == 1000){
		task_cnt_1m = 0;
	}

}


void appTaskfu_10ms(void)
{
	task_cnt_10m++;

	if(school_zone_timer < 10000)
		school_zone_timer++;

	if(turning == TRUE && turn_timer < 10000)
		turn_timer++;

	if(task_cnt_10m == 1000){
		task_cnt_10m = 0;
	}

	if(task_cnt_10m%2 == 0)
	{
		BasicVadcBgScan_run();//적외선 센서 구동
		V2Distance();//적외선센서 출력 거리 계산

		BasicLineScan_run();//라인 읽어들임
		InfineonRacer_detectLane();//라인 검출

		if(IR_Ctrl.basicTest == FALSE){
			#if CODE == CODE_HAND
				InfineonRacer_control(); //dc모터, 서보모터 PID 제어 코드. IR_Motor.Motor0Vol 와 IR_Srv.Angle에 값을 줌
			#elif CODE == CODE_ERT
				IR_Controller_step();
			#else

			#endif
		}

		BasicPort_run();//설정해놓은 GPIO에 지정된 신호 출력
		BasicGtmTom_run();//InfineonRacer_control()에서 주는 IR_Motor.Motor0Vol 와 IR_Srv.Angle값을 바탕으로 모터, 서보 구동

		AsclinShellInterface_runLineScan();
		AsclinShellInterface_runDisScan();
		AsclinShellInterface_runEncScan();
	}
}

void appTaskfu_100ms(void)
{
	task_cnt_100m++;
	if(task_cnt_100m == 1000){
		task_cnt_100m = 0;
	}
#if BOARD == APPLICATION_KIT_TC237
	if(task_cnt_100m % REFRESH_TFT == 0){
		tft_app_run();
	}

#elif BOARD == SHIELD_BUDDY

#endif
}

void appTaskfu_1000ms(void)
{
	task_cnt_1000m++;
	if(task_cnt_1000m == 1000){
		task_cnt_1000m = 0;
	}
}

void appTaskfu_idle(void){
	AsclinShellInterface_run();
#if BOARD == APPLICATION_KIT_TC237
	perf_meas_idle();
#elif BOARD == SHIELD_BUDDY

#endif

}

void appIsrCb_100us(void){
	BasicGpt12Enc_run();
}

