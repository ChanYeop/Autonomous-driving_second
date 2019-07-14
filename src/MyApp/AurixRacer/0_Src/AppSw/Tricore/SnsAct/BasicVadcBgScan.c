/**
 * \file VadcBackgroundScanDemo.c
 * \brief Demo VadcBackgroundScanDemo
 *
 */

/******************************************************************************/
/*----------------------------------Includes----------------------------------*/
/******************************************************************************/

#include <stdio.h>
#include <Cpu/Std/IfxCpu.h>
#include "BasicVadcBgScan.h"
#include "Configuration.h"
/******************************************************************************/
/*-----------------------------------Macros-----------------------------------*/
/******************************************************************************/
#define  ADC_CHN_MAX  2

/******************************************************************************/
/*--------------------------------Enumerations--------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*-----------------------------Data Structures--------------------------------*/
/******************************************************************************/
typedef struct
{
    IfxVadc_Adc vadc;   /* VADC handle*/
    IfxVadc_Adc_Group adcGroup;
    IfxVadc_Adc_Channel adcChannel[ADC_CHN_MAX];
} Basic_VadcBackgroundScan;

/******************************************************************************/
/*------------------------------Global variables------------------------------*/
/******************************************************************************/
Basic_VadcBackgroundScan g_VadcBackgroundScan; /**< \brief Demo information */

#if BOARD == APPLICATION_KIT_TC237
static uint32 adcChannelNum[ADC_CHN_MAX] = {
		3, 4, 8, 9  /* AN15, AN16, AN20, AN21 */
};
#elif BOARD == SHIELD_BUDDY
static uint32 adcChannelNum[ADC_CHN_MAX] = {
		5, 6 /* DAC1, CANRX */
};
#endif


float32 IR_AdcResult[ADC_CHN_MAX];
float32 IR_LPF[ADC_CHN_MAX] = {0}; //�ο� �н� ���� �����
float32 IR_Distance[ADC_CHN_MAX] = {0}; //�Ÿ���

#define FILTER_SIZE 10
#define EMA_a 0.4 //�ο��н����� ���

uint32 filterIx[ADC_CHN_MAX] = {0}; //filter index
float32 filter_buffer[ADC_CHN_MAX][FILTER_SIZE];
float32 filter_sum[ADC_CHN_MAX] = {0};
/******************************************************************************/
/*-------------------------Function Prototypes--------------------------------*/
/******************************************************************************/

/******************************************************************************/
/*------------------------Private Variables/Constants-------------------------*/
/******************************************************************************/

/******************************************************************************/
/*-------------------------Function Implementations---------------------------*/
/******************************************************************************/

/** \brief Demo init API
 *
 * This function is called from main during initialization phase
 */
void BasicVadcBgScan_init(void)
{
    /* VADC Configuration */

    /* create configuration */
    IfxVadc_Adc_Config adcConfig;
    IfxVadc_Adc_initModuleConfig(&adcConfig, &MODULE_VADC);

    /* initialize module */
    IfxVadc_Adc_initModule(&g_VadcBackgroundScan.vadc, &adcConfig);

    /* create group config */
    IfxVadc_Adc_GroupConfig adcGroupConfig;
    IfxVadc_Adc_initGroupConfig(&adcGroupConfig, &g_VadcBackgroundScan.vadc);

#if BOARD == APPLICATION_KIT_TC237
    /* with group 1 */
    adcGroupConfig.groupId = IfxVadc_GroupId_1;
    adcGroupConfig.master  = adcGroupConfig.groupId;
#elif BOARD == SHIELD_BUDDY
    /* with group 5 */
    adcGroupConfig.groupId = IfxVadc_GroupId_5;
    adcGroupConfig.master  = adcGroupConfig.groupId;
#endif


    /* enable background scan source */
    adcGroupConfig.arbiter.requestSlotBackgroundScanEnabled = TRUE;

    /* enable background auto scan */
    adcGroupConfig.backgroundScanRequest.autoBackgroundScanEnabled = TRUE;

    /* enable all gates in "always" mode (no edge detection) */
    adcGroupConfig.backgroundScanRequest.triggerConfig.gatingMode = IfxVadc_GatingMode_always;

    /* initialize the group */
    /*IfxVadc_Adc_Group adcGroup;*/    //declared globally
    IfxVadc_Adc_initGroup(&g_VadcBackgroundScan.adcGroup, &adcGroupConfig);

    /* create channel config */
    uint32                    chnIx;

    IfxVadc_Adc_ChannelConfig adcChannelConfig;

    for (chnIx = 0; chnIx < ADC_CHN_MAX; ++chnIx)
    {
        IfxVadc_Adc_initChannelConfig(&adcChannelConfig, &g_VadcBackgroundScan.adcGroup);

        adcChannelConfig.channelId         = (IfxVadc_ChannelId)(adcChannelNum[chnIx]);
        adcChannelConfig.resultRegister    = (IfxVadc_ChannelResult)(adcChannelNum[chnIx]); // use register #5 and 6 for results
        adcChannelConfig.backgroundChannel = TRUE;

        /* initialize the channel */
        IfxVadc_Adc_initChannel(&g_VadcBackgroundScan.adcChannel[chnIx], &adcChannelConfig);

        /* add to background scan */
        unsigned channels = (1 << adcChannelConfig.channelId);
        unsigned mask     = channels;
        IfxVadc_Adc_setBackgroundScan(&g_VadcBackgroundScan.vadc, &g_VadcBackgroundScan.adcGroup, channels, mask);
    }

    /* start scan */
    IfxVadc_Adc_startBackgroundScan(&g_VadcBackgroundScan.vadc);
}


/** \brief Demo run API
 *
 * This function is called from main, background loop
 */
void BasicVadcBgScan_run(void)
{
    uint32                    chnIx;
    /* wait for valid result */
    volatile Ifx_VADC_RES conversionResult;

        /* check results */
        for (chnIx = 0; chnIx < ADC_CHN_MAX; ++chnIx)
        {
            do
            {
                conversionResult = IfxVadc_Adc_getResult(&g_VadcBackgroundScan.adcChannel[chnIx]);
            } while (!conversionResult.B.VF);

			IR_AdcResult[chnIx] = (float32) conversionResult.B.RESULT / 4095 * 5;

        }
}

/*voltage to distance Conversion*/
void V2Distance(void)
{
	uint32 chnIx;
	float32 temp[ADC_CHN_MAX];

	for (chnIx = 0; chnIx < ADC_CHN_MAX; chnIx++)
	{
		IR_LPF[chnIx] = IR_AdcResult[chnIx] * EMA_a +  IR_LPF[chnIx] * (1 - EMA_a); //�ο� �н� ����

		filterIx[chnIx] = (filterIx[chnIx] + 1) % FILTER_SIZE;
		filter_sum[chnIx] -= filter_buffer[chnIx][filterIx[chnIx]];
		filter_buffer[chnIx][filterIx[chnIx]] = IR_LPF[chnIx];
		filter_sum[chnIx] += filter_buffer[chnIx][filterIx[chnIx]]; //�̵� ��� ����

		temp[chnIx] = filter_sum[chnIx] / FILTER_SIZE;
		IR_Distance[chnIx] = (7.3700) * temp[chnIx] * temp[chnIx] * temp[chnIx] * temp[chnIx]
							+ (-65.0356) * temp[chnIx] * temp[chnIx] * temp[chnIx]
							+ (210.7314) * temp[chnIx] * temp[chnIx]
							+ (-316.0783) * temp[chnIx]
							+ (220.9207);
	}
}
