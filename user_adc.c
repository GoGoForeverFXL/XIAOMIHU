/*
 * user_adc.c
 *
 *  Created on: 2019年12月28日
 *      Author: 22162
 */
/******************************************************************************
 *File Name         :	ad7124_handle.c
 *Copyright         :	BeiJing GoldWind Turbine Co.,Ltd. All Rights Reserved.
 *Create  Date      :	2016/03/07
 *Description       :
 *Revision History:
 *V1.0                 :Liuli  2016/03/07  File Create

******************************************************************************/


/******************************************************************************
*    Include File Section

******************************************************************************/

#include "xparameters.h"	/* XPAR parameters */
#include "xspi.h"		/* SPI device driver */
#include "xspi_l.h"
#include "xil_printf.h"

#include "../drive/ad7124/spi_extra.h"
#include "../drive/ad7124/ad7124.h"
#include "../drive/ad7124/ad7124_handle.h"

#include "../drive/iic/iic_handle.h"

#include  "../user/user_global.h"
/******************************************************************************
*	 Local Macro Define Section
******************************************************************************/
/*
 * The following constants map to the XPAR parameters created in the
 * xparameters.h file. They are defined here such that a user can easily
 * change all the needed parameters in one place.
 */

/*
R = Data * 1000.0 * 2 / ((1 << 24) * GAIN * 1.0);
c = 1000.0 * 2 / ((1 << 24) * GAIN * 1.0)
#define GAIN 4
c = 0.0000298023223876953125*/
#define TEMP_RAW_TO_RESIST_SCALE  0.00002980*2

/*temp huminity clc
 *
 *SensorObjs[i].Temp = -45 + 175.0 * data_L / 65535.0;
 *SensorObjs[i].Huminity = 100.0 * data_H / 65535.0;
 *Temp_c = 175/65535 = 0.00267033
 *Huminty_c = 100/65535 =  0.00152590
 */
#define HUMIDITY_SENSOR_TEMP_SCALE    0.00267033
#define HUMIDITY_SENSOR_TEMP_OFFSET   -45
#define HUMINITY_SENSOR_HUMINTY_SCALE 0.00152590
/******************************************************************************
*	 Local Struct  Define Section
******************************************************************************/


/******************************************************************************
*	 Local Global Variable Define Section
******************************************************************************/


LPF_DESIGN FilterDesign = {0,0,0,0,0,0,350,1};

const  FLOAT32 PT100[400] =
{
 //-149	//-148								                                                //-140
40.14,	40.56,	40.97,	41.39,	41.80,	42.22,	42.63,	43.05,	43.46,	43.88,
44.29,	44.70,	45.12,	45.53,	45.94,	46.36,	46.77,	47.18,	47.59,	48.00,
48.42,	48.83,	49.24,	49.65,	50.06,	50.47,	50.88,	51.29,	51.70,	52.11,
52.52,	52.93,	53.34,	53.75,	54.15,	54.56,	54.97,	55.38,	55.79,	56.19,
56.60,	57.01,	57.41,	57.82,	58.23,	58.63,	59.04,	59.44,	59.85,	60.26,
//-99	//-98								                                                //-90
60.66,	61.07,	61.47,	61.88,	62.28,	62.68,	63.09,	63.49,	63.90,	64.30,
64.70,	65.11,	65.51,	65.91,	66.31,	66.72,	67.12,	67.52,	67.92,	68.33,
68.73,	69.13,	69.53,	69.93,	70.33,	70.73,	71.13,	71.53,	71.93,	72.33,
72.73,	73.13,	73.53,	73.93,	74.33,	74.73,	75.13,	75.53,	75.93,	76.33,
76.73,	77.12,	77.52,	77.92,	78.32,	78.72,	79.11,	79.51,	79.91,	80.31,
//-49	//-48								                                                 //-40
80.70,	81.10,	81.50,	81.89,	82.29,	82.69,	83.08,	83.48,	83.87,	84.27,
84.67,	85.06,	85.46,	85.85,	86.25,	86.64,	87.04,	87.43,	87.83,	88.22,
88.62,	89.01,	89.40,	89.80,	90.19,	90.59,	90.98,	91.37,	91.77,	92.16,
92.55,	92.95,	93.34,	93.73,	94.12,	94.52,	94.91,	95.30,	95.69,	96.09,
96.48,	96.87,	97.26,	97.65,	98.04,	98.44,	98.83,	99.22,	99.61,	100.00,
//01      //2								                                                          //10
100.39,	100.78,	101.17,	101.56,	101.95,	102.34,	102.73,	103.12,	103.51,	103.90,
104.29,	104.68,	105.07,	105.46,	105.85,	106.24,	106.63,	107.02,	107.40,	107.79,
108.18,	108.57,	108.96,	109.35,	109.73,	110.12,	110.51,	110.90,	111.29,	111.67,
112.06,	112.45,	112.83,	113.22,	113.61,	114.00,	114.38,	114.77,	115.15,	115.54,
115.93,	116.31,	116.70,	117.08,	117.47,	117.86,	118.24,	118.63,	119.01,	119.40,
//51      //52								                                                          //60
119.78,	120.17,	120.55,	120.94,	121.32,	121.71,	122.09,	122.47,	122.86, 123.24,
123.63,	124.01,	124.39,	124.78,	125.16,	125.54,	125.93,	126.31,	126.69, 127.08,
127.46,	127.84,	128.22,	128.61,	128.99,	129.37,	129.75,	130.13,	130.52, 130.90,
131.28,	131.66,	132.04,	132.42,	132.80,	133.18,	133.57,	133.95,	134.33, 134.71,
135.09,	135.47,	135.85,	136.23,	136.61,	136.99,	137.37,	137.75,	138.13, 138.51,
//101      //102								                                                   //110
138.88,	139.26,	139.64,	140.02,	140.40,	140.78,	141.16,	141.54,	141.91, 142.29,
142.67,	143.05,	143.43,	143.80,	144.18,	144.56,	144.94,	145.31,	145.69, 146.07,
146.44,	146.82,	147.20,	147.57,	147.95,	148.33,	148.70,	149.08,	149.46, 149.83,
150.21,	150.58,	150.96,	151.33,	151.71,	152.08,	152.46,	152.83,	153.21, 153.58,
153.96,	154.33,	154.71,	155.08,	155.46,	155.83,	156.20,	156.58,	156.95, 157.33,
//151																							//160
157.70, 158.07, 158.45, 158.82, 159.19, 159.56, 159.94, 160.31, 160.68, 161.05,
161.43, 161.80, 162.17, 162.54, 162.91, 163.29, 163.66, 164.03, 164.40, 164.77,
165.14, 165.51, 165.89, 166.26, 166.63, 167.00, 167.37, 167.74, 168.11, 168.48,
168.85, 169.22, 169.59, 169.96, 170.33, 170.70, 171.07, 171.43, 171.80, 172.17,
172.54, 172.91, 173.28, 173.65, 174.02, 174.38, 174.75, 175.12, 175.49, 175.86,
//201
176.22, 176.59, 176.96, 177.33, 177.69, 178.06, 178.43, 178.79, 179.16, 179.53, 				//210
179.89, 180.26, 180.63, 180.99, 181.36, 181.72, 182.09, 182.46, 182.82, 183.19,
183.55, 183.92, 184.28, 184.65, 185.01, 185.38, 185.74, 186.11, 186.47, 186.84,
187.20, 187.56, 187.93, 188.29, 188.66, 189.02, 189.38, 189.75, 190.11, 190.47,
190.84, 191.20, 191.56, 191.92, 192.29, 192.65, 193.01, 193.37, 193.74, 194.10,
};
/******************************************************************************
*	System Global Variable Define Section
******************************************************************************/
FLOAT32 EnTempFiltCal = 0;
INT32 TempNum = 0;
ADC_ENABLE_STRUCT TempChanlEnalbe;
ADC_UINT_STRUCT AdcRawValue;
ADC_STRUCT AdcResistValue;
ADC_STRUCT AdcActValue;
ADC_STRUCT AdcFiltValue;
ADC_STRUCT AdcLastValue;

INT32 HumidityNum = 0;
ADC_ENABLE_STRUCT HumidityChanlEnalbe;

HUMIDITY_ARW_STRUCT HumiRawValue[4];
HUMIDITY_STRUCT HumiActValue;
/******************************************************************************
*	 Local Function Declare Section
******************************************************************************/


/******************************************************************************
*	 System Function Declare Section
******************************************************************************/


/******************************************************************************
*	 Function Define Section
******************************************************************************/
FLOAT32 TempVotlRangeLimit(FLOAT32 value);
FLOAT32 FindPt100Temp(FLOAT32 data);
FLOAT32 LfpTemp(FLOAT32 curr,FLOAT32 last);
INT32 TempAdcInit(void);
INT32 TempAdcRead(UINT32 *rdbuf,int len);
void GetTempResistValue(ADC_STRUCT *resist_value,ADC_UINT_STRUCT *raw_value,ADC_ENABLE_STRUCT enable);
void GetTempActValue(ADC_STRUCT *act_value,ADC_STRUCT *resist_value,ADC_ENABLE_STRUCT enable);
void ADCTempSample(void);
void GetAdcLpfValue(ADC_STRUCT *filt_value,ADC_STRUCT *act_value,ADC_STRUCT *last_value,ADC_ENABLE_STRUCT enable);
int AdcDelay(UINT32 *ms,UINT32 cycle);
/******************************************************************************
 *Name          : TempAdcInit
 *Function       :
 *para            : no
 *return          : no
 *Create         :by Liuli  2015//06/13
******************************************************************************/
INT32 TempAdcInit(void)
{
	INT32 ret1;
	TempChanlEnalbe.enable.chan0 = 1;
	TempChanlEnalbe.enable.chan1 = 1;
	TempChanlEnalbe.enable.chan2 = 1;
	TempChanlEnalbe.enable.chan3 = 1;
	TempChanlEnalbe.enable.chan4 = 1;
	TempChanlEnalbe.enable.chan5 = 1;
	TempChanlEnalbe.enable.chan6 = 1;
	TempChanlEnalbe.enable.chan7 = 1;
	//TempChanlEnalbe.enable.chan8 = 1;
	//TempChanlEnalbe.enable.chan9 = 1;
	//TempChanlEnalbe.enable.chan10 = 1;
	//TempChanlEnalbe.enable.chan11 = 1;
	//TempChanlEnalbe.enable.chan12 = 1;
	//TempChanlEnalbe.enable.chan13 = 1;
	//TempChanlEnalbe.enable.chan14 = 1;
	//TempChanlEnalbe.enable.chan15 = 1;

	ret1 = AdcTempInit(TempChanlEnalbe.all,AdcDelay,&TempNum);
	if(ret1 != 0)
	{
		//输出事件信息，初始化AD芯片错误
		return -1;
	}
	return 0;
}
/******************************************************************************
 *Name          : ADCTempRead
 *Function       :
 *para            : no
 *return          : no
 *Create         :by Liuli  2015//06/13
******************************************************************************/
int AdcDelay(UINT32 *ms,UINT32 cycle)
{
	if(*ms <= 0)
	{
		*ms = 0;
		return 1;
	}
	else
	{
		OSTimeDlyHMSM(0, 0, 0, cycle);
		*ms = *ms - cycle;
		return 0;
	}
}
/******************************************************************************
 *Name          : ADCTempRead
 *Function       :
 *para            : no
 *return          : no
 *Create         :by Liuli  2015//06/13
******************************************************************************/
INT32 TempAdcRead(UINT32 *rdbuf,int len)
{
	INT32 ret1;
	ret1 = AdcTempRead(rdbuf,len);
	if(ret1 != 0)
	{
		return -1;
	}
	return 0;
}

/******************************************************************************
 *Name          : GetTempResistValue
 *Function       :
 *para            : no
 *return          : no
 *Create         :by Liuli  2015//06/13
******************************************************************************/
void GetTempResistValue(ADC_STRUCT *resist_value,ADC_UINT_STRUCT *raw_value,ADC_ENABLE_STRUCT enable)
{
	for(int i=0;i<16;i++)
	{
		if(enable.all&(1<<i))
		{
			resist_value->chan[i] = raw_value->chan[i]*TEMP_RAW_TO_RESIST_SCALE;
		}
	}
}

/******************************************************************************
 *Name          : GetAdcActValue
 *Function       :
 *para            : no
 *return          : no
 *Create         :by Liuli  2015//06/13
******************************************************************************/
void GetTempActValue(ADC_STRUCT *act_value,ADC_STRUCT *resist_value,ADC_ENABLE_STRUCT enable)
{
	for(int i=0;i<16;i++)
	{
		if(enable.all&(1<<i))
		{
			act_value->chan[i] = FindPt100Temp(resist_value->chan[i]);
		}
	}
}
/******************************************************************************
 *Name          : AdcVotlRangeLimit
 *Function       :
 *para            : no
 *return          : no
 *Create         :by Liuli  2015//06/13
******************************************************************************/
FLOAT32 TempVotlRangeLimit(FLOAT32 value)
{
	if(value>=8388607)
	{
		return(value-8388607);
	}
	else
	{
		return(value);
	}
}
/******************************************************************************
 *Name          : FindPt100Temp
 *Function       :
 *para            : no
 *return          : no
 *Create         :by Liuli  2015//06/13
******************************************************************************/
FLOAT32 FindPt100Temp(FLOAT32 data)
{
    INT16  i = 0;
    INT16  temp;
    INT16  Ctop = 0;
    INT16  Cbottom=399;

	//TODO:用电阻≤80.7欧（对应-49度）判定为pt100断线。断线时电阻在0欧左右，为可靠起见，
	//只要电阻对应的负温度不可能达到，就认为断线，故取-49度作为条件。当条件满足时，强制
	//温度为260度，触发过温故障。之所以用过温进行保护，是因为目前程序里没有低温保护，只能
	//临时借用过温故障进行保护。
	if (data <= 80.70)	//@判定为pt100断线 lf:2020/11/12
	{
		temp = 260;
	}
    else if(data >= 194.10)
    {
        temp = 250;
    }
    else
    {
       for(i=199;(Cbottom-Ctop)!=1;)
       {
       	if((data-PT100[i]) > 0)
       	{
       		Ctop=i;
       		i=(Ctop+Cbottom)/2;
       	}
       	else if((data-PT100[i]) < 0 )
       	{
       		Cbottom=i;
       		i=(Ctop+Cbottom)/2;
       	}
       	else
       	{
       		temp=i-149;
       		return(temp);
       	}
       }
       temp=i-149;
       return(temp);
    }
    return(temp);
}
/******************************************************************************
 *Name          :LfpFcHz
 *Function      : 低通滤波器设计(LfpFc350Hz)
                     sample rate = 16KHz,FC = 35Hz,那么
                     k = 6.283*35/16000 = 0.137444680625
                     k1=1/(1+k) = 0.8791636349739
                     k2=1-k1 = 0.1208363650261
 *Para           : no
 *Return        : no
 *Create by Liuli  2014//09/18
******************************************************************************/
FLOAT32 LfpTemp(FLOAT32 curr,FLOAT32 last)
{
    FLOAT32 temvalue[3] = {0,0,0};
    FilterDesign.sample_rate = 16000;
	FilterDesign.curr = curr;
	FilterDesign.last = last;
	FilterDesign.k  = 2*3.1415927*FilterDesign.freq/FilterDesign.sample_rate;
	FilterDesign.k1 = 1/(1+FilterDesign.k);
	FilterDesign.k2 = 1 - FilterDesign.k1;

	temvalue[0] = FilterDesign.curr*FilterDesign.k2;
	temvalue[1] = FilterDesign.last*FilterDesign.k1;
	temvalue[2] = temvalue[0] + temvalue[1];

    return(temvalue[2]);

}

/******************************************************************************
 *Name          : GetAdcLpfValue
 *Function       :
 *para            : no
 *return          : no
 *Create         :by Liuli  2015//06/13
******************************************************************************/
void GetAdcLpfValue(ADC_STRUCT *filt_value,ADC_STRUCT *act_value,ADC_STRUCT *last_value,ADC_ENABLE_STRUCT enable)
{
	for(int i=0;i<16;i++)
	{
		if(enable.all&(1<<i))
		{
			filt_value->chan[i] = LfpTemp(act_value->chan[i],last_value->chan[i]);
			last_value->chan[i]  = filt_value->chan[i];
			act_value->chan[i]   = filt_value->chan[i];
		}
	}
}
/******************************************************************************
 *Name          : ADCTempSample
 *Function       :
 *para            : no
 *return          : no
 *Create         :by Liuli  2015//06/13
******************************************************************************/
void ADCTempSample(void)
{
	AdcTempRead(&AdcRawValue.chan[0],TempNum);
	GetTempResistValue(&AdcResistValue,&AdcRawValue,TempChanlEnalbe);
	GetTempActValue(&AdcActValue,&AdcResistValue,TempChanlEnalbe);
	if(EnTempFiltCal == 1)
	{
		GetAdcLpfValue(&AdcFiltValue,&AdcActValue,&AdcLastValue,TempChanlEnalbe);
	}
}

/******************************************************************************
 *Name          : HumidityTempSensorInit
 *Function       :
 *para            : no
 *return          : no
 *Create         :by Liuli  2015//06/13
******************************************************************************/
INT32 HumiditySensorInit(void)
{
	INT32 ret1;
	HumidityChanlEnalbe.enable.chan0 = 1;
	HumidityChanlEnalbe.enable.chan1 = 1;
	//HumidityChanlEnalbe.enable.chan2 = 1;
	//HumidityChanlEnalbe.enable.chan3 = 1;
	HumidityNum = 0;
	ret1 = HumidityTempSensorInit(HumidityChanlEnalbe.all,AdcDelay,&HumidityNum);
	if(ret1 != 0)
	{
		//输出事件信息，初始化AD芯片错误
		return -1;
	}
	return 0;
}

/******************************************************************************
 *Name          : GetAdcActValue
 *Function       :
 *para            : no
 *return          : no
 *Create         :by Liuli  2015//06/13
******************************************************************************/
void GetHumintyActValue(HUMIDITY_STRUCT *act_value,HUMIDITY_ARW_STRUCT *raw_value,ADC_ENABLE_STRUCT enable)
{
	for(int i=0;i<16;i++)
	{
		if(enable.all&(1<<i))
		{
			act_value->temp[i]     = HUMIDITY_SENSOR_TEMP_OFFSET + raw_value[i].value.temp*HUMIDITY_SENSOR_TEMP_SCALE;
			act_value->humidity[i] = raw_value[i].value.humidity*HUMINITY_SENSOR_HUMINTY_SCALE;
		}
	}
}

/******************************************************************************
 *Name          : GetAdcActValue
 *Function       :
 *para            : no
 *return          : no
 *Create         :by Liuli  2015//06/13
******************************************************************************/
void HumintySensorSample(void)
{
	int err_code[IIC_MAX_CHN_NUM];

	HumiditySensorRead(&HumiRawValue[0].all,HumidityNum , err_code);
	GetHumintyActValue(&HumiActValue,&HumiRawValue[0],HumidityChanlEnalbe);
}
