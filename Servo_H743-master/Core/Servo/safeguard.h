#ifndef SAFEGUARD_H
#define SAFEGUARD_H
#include "current.h"
#include "encoder.h"
#include "main.h"
static const int16_t tempTable[] = {
    3985, 3958, 3930, 3901, 3871, 3840, 3808, 3775, 3741, 3706, 3670, 3633,
    3595, 3556, 3517, 3476, 3434, 3392, 3349, 3305, 3260, 3215, 3168, 3122,
    3074, 3027, 2978, 2930, 2881, 2831, 2781, 2732, 2681, 2631, 2581, 2531,
    2481, 2431, 2381, 2331, 2281, 2232, 2183, 2135, 2087, 2039, 1992, 1945,
    1899, 1853, 1809, 1764, 1721, 1678, 1636, 1594, 1553, 1513, 1474, 1436,
    1398, 1361, 1325, 1289, 1255, 1221, 1188, 1155, 1124, 1093, 1063, 1034,
    1005, 977,  950,  924,  898,  873,  849,  825,  802,  780,  758,  737,
    716,  696,  677,  658,  639,  621,  604,  587,  571,  555,  540,  525,
    510,  496,  483,  469,  457,  444,  432,  420,  409,  398,  387,  377,
    366,  357,  347,  338,  329,  320,  312,  303,  296,  288,  280,  273,
    266,  259,  252,  246,  240,  234};

class Safeguard {
public:
  uint32_t adcValue[10];//通道1 母线电压 通道2 温度
	uint8_t channelNum=2;

	uint16_t errorcode;
  // 三相电流
  float iMax;
  // 温度
  float temp;
  int tempMax;
  // 母线电压 
  float busVoltage_V;
  float udMax;
	float udMin;
	//位置
	int64_t countMax;//最大限制位置


  Safeguard() {}
  inline void init(float iMax, float tempMax, float udMax,float udMin,int64_t countMax) {
    this->iMax = iMax/20.8f;
    this->tempMax = tempMax;
    this->udMax = udMax;
		this->countMax=countMax;
    //    lpFilter.lowpass_init(100, FIX(0.000050)); // 截止频率和采样时间
  }
  //  inline void getCurrent(FeedbackCurrent a) {
  //    iA = a.iA;
  //    iB = a.iB;
  //    iC = a.iC;
  //  }
	
	//计算当前异常状态
  inline uint16_t ErrorCodeGet(float iA,float iB,float iC,int64_t pos_absolute) {

		SCB_InvalidateDCache_by_Addr((uint32_t *)adcValue,channelNum);//sizeof(adcValue)//这句话很关键
		//母线电压 = adcValue[0]/65535*3.0*20
		busVoltage_V = adcValue[0]*0.00091554f;
		//过压
		errorcode &= ~(1u);  // 使用位掩码将最后一位清零
		errorcode |= (busVoltage_V > udMax) ? 1u : 0u;
		//欠压
		errorcode &= ~(1u<<1);  // 使用位掩码将最后第二位清零
		errorcode |= ((busVoltage_V < udMin) ? 1u : 0u)<<1;
    // 温度<<2
    //    temp = std::lower_bound(tempTable, tempTable + 126, adcValue[1],
    //                            std::greater<int>()) -
    //           tempTable;
	  //过流
		errorcode &= ~(1u<<3);  // 使用位掩码将最后第三位清零
		errorcode |= ((fabs(iA)>iMax|fabs(iB)>iMax|fabs(iC)>iMax) ? 1u : 0u)<<3;
		//位置过大
		errorcode &= ~(1u<<4);  // 使用位掩码将最后第四位清零
		errorcode |= ((abs(pos_absolute)>countMax) ? 1u : 0u)<<4;
    return errorcode;
  }

private:

};
#endif // SAFEGUARD_H