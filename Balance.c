#include "include.h"
struct STAND stand;
extern struct Speed Spd;

PID Balance;
Tandem Balance_Tan;
int16 balance_pwm = 0;
int16 balance_pwm_last = 0;
int16 balance_target_gyro_x;
float change_inter = 0;
float target_angle = 0;
float target_angle_last = 0;
extern int16 mpu_gyro_x,mpu_gyro_y,mpu_gyro_z;
extern int16 mpu_acc_x,mpu_acc_y,mpu_acc_z;

uint8 angel_flag = 0;
uint8 count = 0;
float Acc_Filt[5];
float angle_out = 0;
float angle_out_real = 0;
float angle_accelarate = 0;
float angle_accelarate_last = 0;
int32 integer_angle_out;
float substitute = 0;
float angle_tuoluo = 0;
float temperature_gyro_x;
int add=0;

#define ZERO 33
float Target_Angle = 0;
float Target_Angle_Last = 0;

/****************************电机直立环设置************************/

void Motor_Erection(void)
{
  Angle_Target();
  
 // IMUupdate(mpu_gyro_x, mpu_gyro_y,mpu_gyro_z,mpu_acc_x , mpu_acc_y,mpu_acc_z);
  
  angel();
  
  Erection();
}


/*****************************直立环*****************************/

void Erection(void) 
{
  //Balance_PID() ;
  
  float Target_Angular_Speed ;
  
  float Current_Angular_Speed ;
  
  Target_Angular_Speed = Banlance_Tandem_In (integer_angle_out, Target_Angle) ;
  
  Current_Angular_Speed = mpu_gyro_y ;
  
  balance_pwm = Balance_Tandem_Out (Current_Angular_Speed, Target_Angular_Speed) ;
  
  balance_pwm = PWM_Limit(balance_pwm,7000,-7000) ;
  
}

void Balance_PID(void){
  
  Balance.P = 300 ;
//  if(Balance.Error<=7)
//    Balance.D = 400 ;
////  else if(Balance.Error<=4)
////    Balance.D = 500 ;
////  else if(Balance.Error<=7)
////    Balance.D = 300 ;
////  else if(Balance.Error<=15)
////    Balance.D = 200 ;
//  else
   Balance.D = 400 ;
  
  Balance.Error = integer_angle_out - target_angle;
  
  Balance.Error = 0.7 * Balance.Error + 0.3 * Balance.LastError;
    
  Balance.Derror = Balance.Error - Balance.LastError;
  
  balance_pwm = (int16)(-Balance.P * Balance.Error - Balance.D * Balance.Derror) ;
 
  Balance.LastError = Balance.Error;
}

/*
  串级内环
  角度——>角速度
*/
float Banlance_Tandem_In (float current_angle, float target_angle){
  
  float Angular_Speed ;
  
  Balance_Tan.P_In = 40 ;
  
  Balance_Tan.D_In = 25 ;
  
  Balance_Tan.Error_In = current_angle - target_angle ;
  
  Balance_Tan.Derror_In = Balance_Tan.Error_In - Balance_Tan.LastError_In ;
  
  Angular_Speed = Balance_Tan.P_In * Balance_Tan.Error_In - Balance_Tan.D_In * Balance_Tan.Derror_In ;
  
  Balance_Tan.LastError_In = Balance_Tan.Error_In ;  
  
  return Angular_Speed ;
}
/*
  串级外环
  角速度——>PWM
*/
int Balance_Tandem_Out (float current_angular_speed, float target_angular_speed){
  
  int Balance_Pwm ;
  
  Balance_Tan.P_Out =15 ;
  
  Balance_Tan.D_Out = 20 ; 
  
  Balance_Tan.Error_Out = current_angular_speed - target_angular_speed ;
  
  Balance_Tan.Derror_Out = Balance_Tan.Error_Out - Balance_Tan.LastError_Out ;
  
  Balance_Pwm = (int)(Balance_Tan.P_Out * Balance_Tan.Error_Out + Balance_Tan.D_Out * Balance_Tan.Derror_Out) ;
  
  Balance_Tan.LastError_Out = Balance_Tan.Error_Out ;
  
  return Balance_Pwm ;
  
}
/*********************************实际角度计算************************************/
void angel()
{
  float A_angle_Max ;
  
  float A_angle_Min ;
  
  float m_sum = 0;
  
  if(!angel_flag){
    
    angle_out = (atan(mpu_acc_x / (float)(mpu_acc_z)) * 57.3);
    
    angle_accelarate_last = angle_out;
    
    angel_flag = 1;
  }
  else if(mpu_acc_z != 0) {
    
    angle_accelarate = (atan(mpu_acc_x / (float)(mpu_acc_z)) * 57.3);
    
    count = 0;
  }
  Acc_Filt[0]=angle_accelarate;
  
   A_angle_Max = Acc_Filt[0];
   
   A_angle_Min = Acc_Filt[0];
   
   for(uint8 i=4;i>0;i--)
   {
     Acc_Filt[i] = Acc_Filt[i-1];
     
     if(Acc_Filt[i-1]<A_angle_Min) A_angle_Min = Acc_Filt[i];
     
     if(Acc_Filt[i-1]>A_angle_Max) A_angle_Max = Acc_Filt[i];
   }
   if(Acc_Filt[0]<A_angle_Min) A_angle_Min = Acc_Filt[0];
   
   if(Acc_Filt[0]>A_angle_Max) A_angle_Max = Acc_Filt[0];
   
   for(uint8 i=0;i<5;i++)
   {
     m_sum += Acc_Filt[i];
   }    
   angle_accelarate = (m_sum - A_angle_Min - A_angle_Max) / 3.0;
   
   temperature_gyro_x = (float)(-(mpu_gyro_y));
  
  angle_tuoluo += (temperature_gyro_x * 0.006);//量纲

  substitute = (temperature_gyro_x + (angle_accelarate - angle_out_real) * 0.8) * 0.01;//0.005 0.08
  
  angle_out_real += substitute;
  
  integer_angle_out = (int32_t)angle_out_real;
}




//设置目标速度
void Speed_Set()
{
    Spd.Target = 50;

}

/*********************************目标角度设置*********************************/

void Angle_Target(void){
  
  float zero =ZERO ;
  
  Speed_Set();
  
  Spd.Error = (Spd.Target - Spd.feedback_middle);
  
  Spd.Error_Sum += Spd.Error;
  
  Spd.Error_Sum = (int)Range_Limit(Spd.Error_Sum, Spd.feedback_middle/ 5);
  
  if(fabs(Spd.Error) >= 200) change_inter = 0;
  
  else if(fabs(Spd.Error) >= 150 && fabs(Spd.Error) < 200) change_inter = 0.001;
  
  else if(fabs(Spd.Error) >= 100 && fabs(Spd.Error) < 150) change_inter = 0.002;
  
  else if(fabs(Spd.Error) >= 40 && fabs(Spd.Error) < 100) change_inter = 0.003;
  
  else if(fabs(Spd.Error) >= 0 && fabs(Spd.Error) < 40) change_inter = 0.004;

  
  Target_Angle = zero- Spd.Error * 0.10  - (Spd.error - Spd.Error_Last) * 0 - Spd.Error_Sum * change_inter;//0.054
  
  Spd.Error_Last = Spd.Error;
  
  //角度增加限幅
  if(Target_Angle - Target_Angle_Last > 0.4) Target_Angle = Target_Angle_Last + 0.4;
  
  else if (Target_Angle_Last - Target_Angle > 0.4) Target_Angle = Target_Angle_Last - 0.4;
  
  Target_Angle_Last = Target_Angle;
  
  // 角度限幅


  if(Target_Angle < zero - 12 ) Target_Angle = zero - 12;
  
  else if(Target_Angle > zero + 12) Target_Angle = zero + 12;
  
  //if(Target_Angle < 7) Target_Angle = 7;

  Target_Angle_Last = Target_Angle;
}
	
	

/*********************************限幅函数*********************************/
float Range_Limit(float value, float range)
{
  range = fabs(range);
  if(fabs(value) > range){           //微分项限幅
    if(value > 0) value = range;
    else value = -range;
  }
   return value; 
}