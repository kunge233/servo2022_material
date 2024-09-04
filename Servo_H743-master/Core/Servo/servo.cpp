#include "servo.h"

Servo servo;

void Servo::init() {
  // 将电机强拖到电角度0，进行反馈角度初始化
  foc.u_d = 0.08;
  foc.u_q = 0;
  foc.theta = 0;
  foc.ipark();
  foc.svpwm();
  pwm.start();
  pwm.update(foc.t_a, foc.t_b, foc.t_c);
  HAL_Delay(1000);
//  encoder.init(4240 * 4, 5, 2000);
  encoder.init(10000, 4, 2000);
  encoder.reversed = 0; // 换相
  pwm.halt();

  // PID 初始化 24V
//  pid_id.init(1.2965, 0.0238, 0, 0.8);// MaxTorque
//  pid_iq.init(1.2965, 0.0238, 0, 0.8);// MaxTorque
//  pid_speed.init(24.0039, 0.73525, 0, 1); 
//  pid_pos.init(0.00001, 0, 0, 1.0);       // MaxSpeed
	  // PID 初始化 48V
  pid_id.init(2.7185, 0.0508, 0, 0.8);
  pid_iq.init(2.7185, 0.0508, 0, 0.8);
  pid_speed.init(3.0, 0.02850, 0, 1); // MaxTorque
  pid_pos.init(0.0001, 0, 0, 1.0);       // MaxSpeed
//  pid_id.init(6.1435, 0.3708, 0, 0.8);
//  pid_iq.init(6.1435, 0.3708, 0, 0.8);
//  pid_speed.init(3.8867, 0.03225, 0, 1); // MaxTorque
//  pid_pos.init(0.001, 0, 0, 1.0);       // MaxSpeed
//  disturabanceFilter.lowpass_init(5, 0.000050);
//  disturabanceFilter2.lowpass_init(10, 0.000050);

  // LQT 初始化
  lqt_id.init(0.8);
  lqt_iq.init(0.8);
	
	//异常保护初始化 最大电流A 最大温度°最大电压V 最小电压V 最大位置count
	safeguard.init(18,100,25,20,90000000);
	
  is_run = true;
  Statusword_00x6041 = 0b00000001;
}

void Servo::step() {
  if (!is_run)
    return;

  /**
   * CiA DSP 402 状态机
   */
  cia402_step();
  // If Not Operation enabled
  // if ((Statusword_00x6041 & 0b01101111) != 0b00100111) {
  //   pwm.halt();
  //   return;
  // }

  /**
   * 外部接口转换
   */
  external_interface();

  /**
   * Get feedback
   */
  current.get();
  encoder.get();
  foc.i_a = current.i_a;
  foc.i_b = current.i_b;
  foc.theta = encoder.theta;
  foc.clarke();
  foc.park();

  /**
   * 对象字典值映射
   */
  // Error_code_00x603F = err;
  Modes_of_operation_display_00x6061 = Modes_of_operation;
  Position_actual_value_00x6064 = encoder.pos_absolute;
  Velocity_actual_value_00x606C = encoder.speed_rpm;
  Torque_actual_value_00x6077 = pid_id.u;
	
  /**
   * Safe
   */
	 Error_code = safeguard.ErrorCodeGet(current.i_a,current.i_b,current.i_c,encoder.pos_absolute);
   if (Error_code!=0) {
  //   Modes_of_operation = 254;
   }

  switch (Modes_of_operation) {
  // 刹车模式
  case 0:
    pwm.halt();
    break;

  // Profile Position Mode
  case 1:
    position_loop();
    speed_loop();
    current_loop();
    break;

  // Profile Velocity Mode
  case 3:
    speed_loop();
    current_loop();
    break;

  // Torque Profile Mode
  case 4:
    current_loop();
    break;

  // 开环
  case 5:
    open_loop();
    break;

  // Homing Mode
  case 6:
    pos_d = 0;
    position_loop();
    speed_loop();
    current_loop();
    break;
  // 位置初始化
  case 7:
    speed_d = -0.25;
    speed_loop();
    current_loop();
    //		if(HAL_GPIO_ReadPin(CLOSE_SWITCH_GPIO_Port,CLOSE_SWITCH_Pin)==0){
    //			Modes_of_operation=254;
    //		}
    break;
  // 电阻电感辨识
  case 10:
    resistance_inductance_identification();
    break;

  case 11:
    open_loop_frequency_domain_identification();
    break;

  // 电流频率响应
  case 12:
    id_d = wave.step();
    iq_d = 0;
    current_loop();
    break;

  // 速度频率响应
  case 13:
    speed_d = wave.step();
    speed_loop();
    current_loop();
    break;

  // 位置频率响应
  case 14:
    pos_d = wave.step();
    position_loop();
    speed_loop();
    current_loop();
    break;
    //速度环P_PI切换
  case 15:
    speed_loop();
    current_loop();
    break;
    //速度环P_PI切换
  case 16:
    speed_loop();
    current_loop();
    break;
  case 17:
    speed_loop();
    current_loop();
    break;
	//固定加速度
	case 18:
		PVM();
		current_loop();
	break;
	//固定速度
	case 19:
		PPM();
		current_loop();
	break;
	//轮廓力矩模式
	case 20:
		PTM();
	break;
  // lqr电流环
  case 22:
    lqr_current_loop();
    break;
  //下位机电阻辨识
  case 30:
    resistance_identification();
    break;
  case 31:
    inductance_identification();
    break;
  // 软重启
  case 255:
    __set_FAULTMASK(1);
    HAL_NVIC_SystemReset();
    break;

  // 重置
  case 254:
    pid_id.reset();
    pid_iq.reset();
    pid_speed.reset();
    pid_pos.reset();
    lqt_id.reset();
    lqt_iq.reset();
    id_d = 0;
    iq_d = 0;
    speed_d = 0;
    pos_d = encoder.pos_absolute;
    wave.reset();
    mls.reset();
    reset_identification();
    Modes_of_operation = 0;
		PVM_Vtar=0;
		PVM_Vcur=0;
		PVM_temp=0;
    break;
  }
}

void Servo::open_loop() {
  ////////// Calculate
  // Controller
  foc.u_d = 0.05;
  foc.u_q = 0;
  static float open_loop_theta = 0;
  foc.theta = open_loop_theta;
  open_loop_theta += 2 * M_PI / FOC_CTRL_FREQ;
  if (open_loop_theta >= 2 * M_PI)
    open_loop_theta -= 2 * M_PI;

  ////////// OutPut
  foc.ipark();
  foc.svpwm();

  pwm.update(foc.t_a, foc.t_b, foc.t_c);
  // pwm.update(0.6, 0.5, 0.5);
}

void Servo::current_loop() {
  ////////// Calculate
  // Controller

//  static float temp = 0;
//  //	static unsigned char n=0;
//  y_n_1 = y_n;
//  //	//wm=encoder.speed_rpm * 2 *3.1415926 / 60;
//  wm = encoder.speed_rpm * 0.1047f;
//  //
//  //
//  //	//z1=(-foc.i_q*125/6A*1.5*pn*flux+wm*Br+(wm*k3+z2))*k3/J;
//  u_n = (-foc.i_q * 1.8266f + wm * Br + (wm * k3 + y_n_1)) * k3 * J_;
//  //
//  y_n = y_n_1 + Ts * K_Ts * u_n;
//  //
//  if (y_n > 10000000.0f)
//    y_n = 10000000.0f;
//  if (y_n < -10000000.0f)
//    y_n = -10000000.0f;

//  //	n++;
//  //	temp+=y_n+wm*k3;
//  //	if(n==10){
//  //		TL=temp/10.0f;
//  //		n=0;
//  //		temp=0;
//  //	}
//  //
//  TL = y_n + wm * k3;
//  // TL=disturabanceFilter.calc2(TL);
//  //
//  //	if(TL<0.08&&TL>-0.08)
//  //				iq_=0;
//  //	else
//  iq_ = TL * Kt / 20.8f;
//  iq_ = disturabanceFilter2.calc2(iq_);
//  if (iq_ > 0.9)
//    iq_ = 0.9;
//  if (iq_ < -0.9)
//    iq_ = -0.9;
//  //	if(Modes_of_operation==3)
//  //		iq_d+=0.8*iq_;
//  if (iq_d > 1)
//    iq_d = 1;
//  if (iq_d < -1)
//    iq_d = -1;

  foc.u_d = pid_id.step(id_d - foc.i_d);
  foc.u_q = pid_iq.step(iq_d - foc.i_q);

  ////////// OutPut
  foc.ipark2();
  foc.svpwm();

  pwm.update(foc.t_a, foc.t_b, foc.t_c);
  sample.save_data();
}

void Servo::speed_loop() {
  id_d = 0;
  //加入扰动
  //正弦扰动
  //	float amp=100.0f/2000.0f;//幅值
  //	float freq=100;//频率

  //	float frao;
  //  static int k=0;
  //	frao=amp * fast_sin(freq * 2 * M_PI / FOC_CTRL_FREQ * k);
  //	k++;
  //	if(k>=FOC_CTRL_FREQ) k=0;

  //  iq_d = pid_speed.step(speed_d - encoder.speed);
  if (Modes_of_operation == 3)
    iq_d = pid_speed.step(speed_d - encoder.speed);
  else if (Modes_of_operation == 15)
    iq_d = pid_speed.step2(speed_d - encoder.speed);
  else if (Modes_of_operation == 16)
    iq_d = pid_speed.P_PI_Switch_step2(speed_d - encoder.speed, encoder.speed,
                                       speed_d);
  else if (Modes_of_operation == 17)
    iq_d = pid_speed.P_PI_Switch_step(speed_d - encoder.speed, speed_d);
  else {
    iq_d = pid_speed.step(speed_d - encoder.speed);
  }
  for (int i = 0; i < n_notch_filter; i++) {
    iq_d = notch_filter[i].calc(iq_d);
  }
}

void Servo::PTM() {
	if(PTM_Ttar*PTM_TtarM!=iq_d*20.8f){
		PTM_Ttar=iq_d*20.8f;
		PTM_Tcur=foc.i_q*20.8f;
		PTM_TtarM=sgn(PTM_Ttar);
		PTM_TcurM=sgn(PTM_Tcur);
		PTM_t=0;
		//将速度都变成正数，方便后续计算
		PTM_Ttar=fabs(PTM_Ttar);
		PTM_Tcur=fabs(PTM_Tcur);
		PTM_state=1;
	}
	switch(PTM_state){
		case 0://IDLE
			PTM_t=0;
		break;
		case 1://规划路径
			if(PTM_TcurM==0||PTM_TcurM==PTM_TtarM){//速度方向一致
					if(sgn(PTM_Ttar-PTM_Tcur)>=0){//加速
						PTM_t1=0;
						PTM_t1M=0;
						PTM_t2=(PTM_Ttar-PTM_Tcur)*PPM_T/PTM_acc;
						PTM_t2M=PTM_TtarM;
					}
					else {//减速
						PTM_t1=fabs(PTM_Ttar-PTM_Tcur)*PPM_T/PTM_acc;
						PTM_t1M=PTM_TtarM;
						PTM_t2=0;
						PTM_t2M=0;
					}
			}
			else{//速度方向不一致，需要减速
				PTM_t1=PTM_Tcur*PPM_T/PTM_acc;
				PTM_t1M=PTM_TcurM;
				PTM_t2=PTM_Ttar*PPM_T/PTM_acc;
				PTM_t2M=PTM_TtarM;
			}
			//将时间累积起来，后面方便分情况
			PTM_t2=PTM_t2+PTM_t1;
			PTM_state=2;
		break;
		case 2://正在执行
			if(PTM_t<PTM_t1){//减速
				PTM_Tcur=PTM_Tcur-PTM_acc/PPM_T;
				PTM_temp=PTM_t1M*PTM_Tcur/20.8f;
			}
			else if(PTM_t>=PTM_t1&&PTM_t<PTM_t2){//加速
				PTM_Tcur=PTM_Tcur+PTM_acc/PPM_T;
				PTM_temp=PTM_t2M*PTM_Tcur/20.8f;
			}
			else{//匀速
				PTM_temp=PTM_TtarM*PTM_Ttar/20.8f;
				PTM_state=0;
			}
			PTM_t++;
		break;
	}
	TL=PTM_temp;
  foc.u_d = pid_id.step(id_d - foc.i_d);
  foc.u_q = pid_iq.step(PTM_temp - foc.i_q);

  ////////// OutPut
  foc.ipark2();
  foc.svpwm();

  pwm.update(foc.t_a, foc.t_b, foc.t_c);
  sample.save_data();
}
void Servo::PVM() {
	if(PVM_Vtar*PVM_VtarM!=speed_d*encoder.speed_rpm_max){
		PVM_Vtar=speed_d*encoder.speed_rpm_max;
		PVM_Vcur=encoder.speed_rpm;
		PVM_VtarM=sgn(PVM_Vtar);
		PVM_VcurM=sgn(PVM_Vcur);
		PVM_t=0;
		//将速度都变成正数，方便后续计算
		PVM_Vtar=fabs(PVM_Vtar);
		PVM_Vcur=fabs(PVM_Vcur);
		PVM_state=1;
	}
	switch(PVM_state){
		case 0://IDLE
			PVM_t=0;
		break;
		case 1://规划路径
			if(PVM_VcurM==0||PVM_VcurM==PVM_VtarM){//速度方向一致
					if(sgn(PVM_Vtar-PVM_Vcur)>=0){//加速
						PVM_t1=0;
						PVM_t1M=0;
						PVM_t2=(PVM_Vtar-PVM_Vcur)*PPM_T/PPM_acc;
						PVM_t2M=PVM_VtarM;
					}
					else {//减速
						PVM_t1=fabs(PVM_Vtar-PVM_Vcur)*PPM_T/PPM_dec;
						PVM_t1M=PVM_VtarM;
						PVM_t2=0;
						PVM_t2M=0;
					}
			}
			else{//速度方向不一致，需要减速
				PVM_t1=PVM_Vcur*PPM_T/PPM_dec;
				PVM_t1M=PVM_VcurM;
				PVM_t2=PVM_Vtar*PPM_T/PPM_acc;
				PVM_t2M=PVM_VtarM;
			}
			//将时间累积起来，后面方便分情况
			PVM_t2=PVM_t2+PVM_t1;
			PVM_state=2;
		break;
		case 2://正在执行
			if(PVM_t<PVM_t1){//减速
				PVM_Vcur=PVM_Vcur-PPM_dec/PPM_T;
				PVM_temp=PVM_t1M*PVM_Vcur/encoder.speed_rpm_max;
			}
			else if(PVM_t>=PVM_t1&&PVM_t<PVM_t2){//加速
				PVM_Vcur=PVM_Vcur+PPM_acc/PPM_T;
				PVM_temp=PVM_t2M*PVM_Vcur/encoder.speed_rpm_max;
			}
			else{//匀速
				PVM_temp=PVM_VtarM*PVM_Vtar/encoder.speed_rpm_max;
				PVM_state=0;
			}
			PVM_t++;
		break;
	}
	TL=PVM_temp;
	iq_d = pid_speed.step(PVM_temp - encoder.speed);

}
void Servo::test(){
	  static int i, t1,t2,t3;
	  float temp;
		t1=32000;
		t2=43500;
		t3=63500;
		if(i<t1){
			temp=0+800.0f/2000.0f*i/t1;
			i++;
		}
		else if(t1<=i&&i<=t2){
			temp=800.0f/2000.0f;
			i++;
		}
		else if(t2<i&&i<t3){
			temp=800.0f/2000.0f-800.0f/2000.0f*(i-t2)/(t3-t2);
			i++;
		}
		else if(i>=t3){
			temp=0;
		}
		TL=temp;
		iq_d = pid_speed.step(temp - encoder.speed);
}
void Servo::PPM() {
		uint64_t temp = 0;
		if(PPM_tarpos!=pos_d){
			if(PPM_mode==0){//连续模式
				PPM_tarpos=pos_d;
				PPM_curpos=encoder.pos_absolute;
				PPM_state=1;
				//清空队列
				PPM_posQueuerear=PPM_posQueueFront=0;
			}
			if(PPM_mode==1){//单点模式
				if(PPM_state==2){//上一阶段还没完成把位置存入队列
					//判断是否满队
					if((PPM_posQueuerear+1)%10!=PPM_posQueueFront){
						//队列+1
						PPM_posQueue[PPM_posQueueFront]=pos_d;
						PPM_tarpos=pos_d;
						PPM_curpos=encoder.pos_absolute;
						PPM_posQueueFront=(PPM_posQueueFront+1)%10;
					}
				}
				else{//直接执行
					//队列不管
					PPM_tarpos=pos_d;
					PPM_curpos=encoder.pos_absolute;
					PPM_state=1;
				}
			}
		}
		switch(PPM_state){
			case 0://IDLE
				PPM_t=0;
			break;
			case 1://规划路径
				PPM_t=0;
				PPM_acck=PPM_acc/encoder.lines*60;
				PPM_deck=PPM_dec/encoder.lines*60;
				PPM_epos=PPM_tarpos-PPM_curpos;
				PPM_eposM=sgn(PPM_epos);
				//PPM_VcurM=sgn(PPM_Vcur);
				PPM_VcurM=sgn(speed_d);
				if((PPM_VcurM==0||PPM_eposM==PPM_VcurM) && abs(PPM_epos)>=(PPM_Vcur*PPM_Vcur/2/PPM_deck))//不用刹车的情况
				{
					//无减速阶段
					PPM_t1=0;
					PPM_t1M=0;
					PPM_t2M=PPM_eposM;
					temp = (PPM_Vel*PPM_Vel-PPM_Vcur*PPM_Vcur)/2/PPM_acck+(PPM_Vel*PPM_Vel/2/PPM_deck);
					if(abs(PPM_epos)>=temp)//有匀速阶段的情况
					{
						//加速时间
						PPM_t2=(PPM_Vel-PPM_Vcur)*PPM_T/PPM_acc;
						//匀速时间
						PPM_t3=(abs(PPM_epos)-temp)*60*PPM_T/encoder.lines/PPM_Vel;
						//减速时间
						PPM_t4=PPM_Vel*PPM_T/PPM_dec;
					}
					else//无匀速阶段的情况
					{
						PPM_Vel_=sqrtf(PPM_acck*PPM_deck*2*abs(PPM_epos)/(PPM_acck+PPM_deck));
						//加速时间
						PPM_t2=(PPM_Vel_-PPM_Vcur)*PPM_T/PPM_acc;
						//匀速时间
						PPM_t3=0;
						//减速时间
						PPM_t4=PPM_Vel_*PPM_T/PPM_dec;
					}
				}
				else{
					//先计算刹车时间
					PPM_t1=PPM_Vcur*PPM_T/PPM_dec;
					PPM_t1M=PPM_VcurM;
					PPM_t2M=-PPM_t1M;
					//加上刹车距离
					PPM_epos=abs(PPM_epos)+PPM_Vcur*PPM_Vcur/2/PPM_deck;
					//刹完车后速度为0
					temp = (PPM_Vel*PPM_Vel)/2/PPM_acck+(PPM_Vel*PPM_Vel/2/PPM_deck);
					if(PPM_epos>=temp)//有匀速阶段的情况
					{
						//加速时间
						PPM_t2=PPM_Vel*PPM_T/PPM_acc;
						//匀速时间
						PPM_t3=(PPM_epos-temp)*60*PPM_T/encoder.lines/PPM_Vel;
						//减速时间
						PPM_t4=PPM_Vel*PPM_T/PPM_dec;
					}
					else//无匀速阶段的情况
					{
						PPM_Vel_=sqrtf(PPM_acck*PPM_deck*2*PPM_epos/(PPM_acck+PPM_deck));
						//加速时间
						PPM_t2=PPM_Vel_*PPM_T/PPM_acc;
						//匀速时间
						PPM_t3=0;
						//减速时间
						PPM_t4=PPM_Vel_*PPM_T/PPM_dec;
					}
				}
				//将时间累积起来，后面方便分情况
				PPM_t2=PPM_t2+PPM_t1;
				PPM_t3=PPM_t3+PPM_t2;
				PPM_t4=PPM_t4+PPM_t3;
				PPM_state=2;
			break;
			case 2://正在执行
				if(PPM_t<PPM_t1){//减速阶段
					PPM_Vcur=PPM_Vcur-PPM_dec/PPM_T;
					speed_d=PPM_t1M*PPM_Vcur/encoder.speed_rpm_max;
				}
				else if(PPM_t>=PPM_t1 && PPM_t<PPM_t2){//加速阶段
					PPM_Vcur=PPM_Vcur+PPM_acc/PPM_T;
					speed_d=PPM_t2M*PPM_Vcur/encoder.speed_rpm_max;
				}
				else if(PPM_t>=PPM_t2 && PPM_t<PPM_t3){//匀速阶段
					PPM_Vcur=PPM_Vel;
					speed_d=PPM_t2M*PPM_Vcur/encoder.speed_rpm_max;
				}
				else if(PPM_t>=PPM_t3 && PPM_t<PPM_t4){//减速阶段
					PPM_Vcur=PPM_Vcur-PPM_dec/PPM_T;
					speed_d=PPM_t2M*PPM_Vcur/encoder.speed_rpm_max;
				}
				else {//结束阶段
					PPM_Vcur=0;
					speed_d=0;
					PPM_state=0;
					//判断队列是否为空，若不为空队列-1
					if(PPM_posQueuerear!=PPM_posQueueFront){
						pos_d=PPM_posQueue[PPM_posQueuerear];
						PPM_posQueuerear=(PPM_posQueuerear+1)%10;
					}
				}
				PPM_t++;
			break;
		}
		TL=speed_d;
		iq_d = pid_speed.step(speed_d - encoder.speed);
//  speed_d = pid_pos.step(posSet - encoder.pos_absolute);
}

void Servo::position_loop() {
  speed_d = pid_pos.step(pos_d - encoder.pos_absolute);
}

void Servo::resistance_inductance_identification() {
  foc.t_a = 0.5;
  foc.t_b = 0.5;
  pwm.update(foc.t_a, foc.t_b, foc.t_c);
  sample.save_data();
}
void Servo::resistance_identification() {

  if (res_j < n_sample) {
    //电压设置
    foc.t_a = 0.5;
    foc.t_b = 0.5;
    foc.t_c = -delta_u_c * res_i + 0.5;
    pwm.update(foc.t_a, foc.t_b, foc.t_c);
    //记录最后res_m个点的电流稳态值
    if (res_j >= n_sample - res_m)
      avg_iC[res_i] += current.i_c;
  }
  res_j++;
  if (res_j >= n_sample && res_j < n_sample2) {
    //留时间使电机电流降低
    pwm.halt();
  } else if (res_j >= n_sample2) {
    //计算均值
    avg_iC[res_i] /= res_m;
    //下一轮重置条件
    res_j = 0;
    res_i++;
  }
  if (res_i > n_regression) {
    //数据记录完成,计算最终电阻
    double sum_xy = 0, sum_x2 = 0, x_bar = 0, y_bar = 0;
    for (int i = 1; i <= n_regression; i++) {
      double x = delta_u_c * i;
      x_bar += x;
      y_bar += avg_iC[i];
      sum_xy += x * avg_iC[i];
      sum_x2 += x * x;
    }
    x_bar /= n_regression;
    y_bar /= n_regression;
    double k = (sum_xy - n_regression * x_bar * y_bar) /
               (sum_x2 - n_regression * x_bar * x_bar);
    Rs = abs(k_correct / k);
    Modes_of_operation = 254;
  }
}
void Servo::inductance_identification() {
  if (ind_i == 0) { //第一次先吸住 并且计算稳态值
    if (ind_j < n_sample * 4) {
      foc.t_a = 0.5;
      foc.t_b = 0.5;
      foc.t_c = 0.55;
      pwm.update(foc.t_a, foc.t_b, foc.t_c);
      //记录最后ind_m个点的电流稳态值
      if (ind_j >= n_sample * 4 - ind_m)
        ind_avg += current.i_c;
    }
    ind_j++;
    if (ind_j >= n_sample * 4 && ind_j < n_sample2 * 4) {
      //留时间使电机电流降低
      pwm.halt();
    } else if (ind_j >= n_sample2 * 4) {
      //下一轮重置条件
      ind_j = 0;
      ind_i++;
      riseTime = 0;
      //计算稳态值
      ind_avg = ind_avg / ind_m * 0.632;
    }
  } else {
    if (ind_j < n_sample) {
      foc.t_a = 0.5;
      foc.t_b = 0.5;
      foc.t_c = 0.55;
      pwm.update(foc.t_a, foc.t_b, foc.t_c);
      // sampleData1[ind_j]=current.i_c;
      //计算时间点
      if (lastCurIc < ind_avg && current.i_c > ind_avg) {
        riseTime += 50 * ind_j;
        ind_j = n_sample - 1; //只记录一次数据，强行进入二阶段
      }
      lastCurIc = current.i_c;
    }
    ind_j++;
    if (ind_j >= n_sample && ind_j < n_sample2) {
      //留时间使电机电流降低
      pwm.halt();
    } else if (ind_j >= n_sample2) {
      //下一轮重置条件
      ind_j = 0;
      ind_i++;
      lastCurIc = 0;
    }
    if (ind_i > turn) {
      riseTime = riseTime / turn;
      Ls = riseTime * Rs / 2;
      Modes_of_operation = 254;
    }
  }
}
void Servo::reset_identification() {
  memset(avg_iC, 0, sizeof(avg_iC));
  res_i = 1;
  res_j = 0;
  ind_i = 0;
  ind_j = 0;
  ind_avg = 0;
}
void Servo::open_loop_frequency_domain_identification() {
  ////////// Calculate
  // Controller
  // id_d = 0.1f * mls.step();
  // foc.u_d = pid_id.step(id_d);
  foc.u_d = 0.1f * mls.step();
  foc.u_q = 0;
  foc.theta = 0;

  ////////// OutPut
  foc.ipark();
  foc.svpwm();
  pwm.update(foc.t_a, foc.t_b, foc.t_c);

  ////////// Save
  if (mls.k >= MLS_SIZE) {
    Modes_of_operation = 254;
  } else {
    *(float *)&sample_p1_data[mls.k - 1] = foc.i_d;
    *(float *)&sample_p2_data[mls.k - 1] = foc.u_d;
  }
}

void Servo::lqr_current_loop() {
  ////////// Calculate
  // Controller
  foc.u_d = lqt_id.step(foc.i_d, id_d);
  foc.u_q = lqt_id.step(foc.i_q, iq_d);

  ////////// OutPut
  foc.ipark2();
  foc.svpwm();
  pwm.update(foc.t_a, foc.t_b, foc.t_c);
  sample.save_data();
}

void Servo::external_interface() {
//  iq_d = (float)Target_torque_00x6071 / (125.0f / 6) / 1000.0f;
//  pos_d = (float)Target_position_00x607A;
//  speed_d = (float)Target_velocity_00x60FF / 2000.0f;
	Error_code = Error_code_00x603F;
  Modes_of_operation = Modes_of_operation_00x6060;
//  pid_pos.u_max = (float)Max_motor_speed_00x6080 / 2000.0f;
//  pid_speed.u_max = (float)Max_torque_00x6072 / (125.0f / 6) / 1000.0f;
  //pid_iq.u_max = (float)Max_torque_00x6072 / (125.0f / 6) / 1000.0f;
}

extern "C" void servo_init() { servo.init(); 
	HAL_ADC_Start_DMA(&hadc1,(uint32_t *)&servo.safeguard.adcValue,servo.safeguard.channelNum);}

extern "C" void servo_step() { servo.step(); }
