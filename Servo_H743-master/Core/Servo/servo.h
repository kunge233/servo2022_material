#pragma once
#include "MyApplication.h"
#include "cia402.h"
#include "current.h"
#include "encoder.h"
#include "foc.h"
#include "kf.h"
#include "lqt.h"
#include "mls.h"
#include "pid.h"
#include "pwm.h"
#include "sample.h"
#include "wave.h"
#include <cstdlib>
#include "safeguard.h"
#include "math.h"
#define sgn(x) ((x > 0) ? 1 : ((x < 0) ? -1 : 0))
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;

extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim8;

class Servo {
public:
  // 硬件
  Current<float, &hadc1, &hadc2, &hadc3> current;
  Encoder encoder;
  PWM<float, &htim8> pwm;

  // 控制器
  FOC<float> foc;
  PID<float> pid_id;
  PID<float> pid_iq;
  PID<float> pid_speed;
  PID<float> pid_pos;

  LQT<float> lqt_id;
  LQT<float> lqt_iq;

  // 采样
  Sample sample;

  // 电流、速度、位置的设定值
  float id_d;
  float iq_d;
  float speed_d;
  float pos_d;
  // 波形生成
  Wave wave;

  // 频域辨识
  MLS mls;
	
	//UINT16 &Error_code=Error_code_00x603F;
	UINT16 Error_code;
  UINT16 &Controlword = Controlword_00x6040;
  //INT8 &Modes_of_operation = Modes_of_operation_00x6060;
	INT8 Modes_of_operation;
  UINT16 &Max_torque = Max_torque_00x6072;
  UINT32 &Max_motor_speed = Max_motor_speed_00x6080;
  UINT16 &Touch_probe_function = Touch_probe_function_00x60B8;

  bool is_run;

  __IO Servo *save;

  // 初始化
  void init();

  // 单步运算
  void step();

  // 开环
  void open_loop();

  // 电流环
  void current_loop();
	void PTM();
	uint8_t PTM_state;
	uint64_t PTM_t;
	uint64_t PTM_t1;
	uint64_t PTM_t2;
	float PTM_eTor;
	float PTM_Tcur;
	float PTM_Ttar;
	float PTM_temp;
	float PTM_acc=10.0f;
	int8_t PTM_TcurM;
	int8_t PTM_TtarM;
	int8_t PTM_t1M;
	int8_t PTM_t2M;
	
  // 速度环
  void speed_loop();
	void PVM();
	
	uint8_t PVM_state;
	uint64_t PVM_t;
	uint64_t PVM_t1;
	uint64_t PVM_t2;
	float PVM_eVel;
	float PVM_Vcur;
	float PVM_Vtar;
	float PVM_temp;
	int8_t PVM_VcurM;
	int8_t PVM_VtarM;
	int8_t PVM_t1M;
	int8_t PVM_t2M;
	
  // 位置环
	void PPM();
	void test();
  void position_loop();
	uint8_t PPM_mode=0;//0连续 1单点
	uint8_t PPM_state;
	int64_t PPM_epos;
	int64_t PPM_tarpos;
	int64_t PPM_curpos;
	float PPM_Vcur;
	float PPM_Vel=800;//默认800rpm
	float PPM_Vel_;//无匀速情况的最大速度
	float PPM_acc=500;//rpm/s
	float PPM_dec=800;//rpm/s
	float PPM_acck;//单位统一版
	float PPM_deck;
	float PPM_T=20000.0f;//周期时间1s
	uint64_t PPM_t;
	uint64_t PPM_t1;
	uint64_t PPM_t2;
	uint64_t PPM_t3;
	uint64_t PPM_t4;
	int8_t PPM_eposM;
	int8_t PPM_VcurM;
	int8_t PPM_t1M;
	int8_t PPM_t2M;
	int64_t PPM_posQueue[10];//单点模式下的位置队列
	int8_t PPM_posQueueFront=0;//队头
	int8_t PPM_posQueuerear=0;//队尾
	
  // 电阻电感辨识
  void resistance_inductance_identification();
  void resistance_identification();
  void inductance_identification();
  void reset_identification();
  int n_sample = 400;
  int n_sample2 = 800;
  // 可自定义，线性回归的点数和每次上升的电压
  int n_regression = 30;
  double avg_iC[40] = {0};
  double delta_u_c = 0.05 / n_regression;
  int res_i = 1;       //外层
  int res_j = 0;       //内层
  int res_m = 50;      //取最后50个点
  double u_max = 48.0; // 电压
  double i_max = 125.0 / 6;
  // double k_correct = 2.0 / 3 * u_max / i_max;
  double k_correct = u_max / i_max;
  double Rs;

  int turn = 10;
  int ind_i = 0;  //外层
  int ind_j = 0;  //内层
  int ind_m = 50; //取最后50个点
  double lastCurIc;
  double ind_avg = 0;
  int riseTime = 0;
  double Ls;
  // 开环频域辨识
  void open_loop_frequency_domain_identification();

  // LQR电流环
  void lqr_current_loop();

  // 外部接口转换，如EtherCAT
  void external_interface();
  //滤波器
  int n_notch_filter = 0;
  SOSFilter notch_filter[5];
  SOSFilter disturabanceFilter;
  SOSFilter disturabanceFilter2;

  //负载转矩辨识
  //电机的其他参数
  unsigned char pn = 4;
  // 电机的各种参数
  float flux = 0.0146135;
  float Br = 0.0003882; // 0.0003882;
  float J = 0.0000342;
  float J_ = 292397.6f; // 29239.76f;
  float x_n = 0;
  float x_n_1 = 0;
  float u_n = 0;
  float u2_n = 0;
  float y_n = 0;
  float y_n_1 = 0;
  float z_n = 0;
  float wm = 0;
  float TL = 0;
  float iq_ = 0;
  float k3 = -0.1f;
  float Kt = 2.0f / (3.0f * pn * flux);
  float Ts = 0.000050;
  float K_Ts = 1.0f;
	//异常保护
	Safeguard safeguard;
};

extern Servo servo;
