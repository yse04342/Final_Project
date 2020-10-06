//////////////////////////////////////////////////////////////////////
// USART RX Interrupt 
// UART5: TX pin: PC12, RX pin: PD2 
// TX: Polling ���, RX: Interrupt ��� 
// ���ڸ� TX�� ���� PC(Hyper terminal)�� �����ϰ�, 
// PC���� ������ ���ڸ� �޾� LCD�� ǥ��

//              2020, 06, 01���� ���� 
// 1 . ���ڱ� �޾ƿ��� ���� ,
// 2. ���� timer 1 , ������ timer2  PC6 , PG1,  PG2,  PG3 - ����  PC7, PG4,5,6, 
//

//NVIC_SystemReset();                              //�����ϴ� ��ư 

// GPS �� ������ -> �������� �̵��� // ����  �� ���� , �浵 �� ���� -> �̷� �� ���������� 90�� ȸ��  
// GPS �� ���� -> ������ �̵��� // ������ ���� , �浵 �� ����  -> �̷��� �������� 90�� ȸ�� 

// GPS  ���� �浵 �׽�Ʈ �ڵ� 
// 1. ���� : 37.340772
//    �浵 : 126.732717                                                   
//2. ���� : 37.340746
//    �浵 : 126.732760                                   ���� ���� 
//3. ���� : 37.340727
//    �浵 : 126.732743
//4. ���� : 37.340754
//    �浵 : 126.732700 
//
//
// Z_deg = �տ� ���� ���� �����ϰ� �ϱ�. 
// 400 , 640 �϶� ���� ������� ��                     
//////////////////////////////////////////////////////////////////////
#include "stm32f4xx.h"
#include "stm32f4xx_spi.h"
#include "defines.h"
#include "tm_stm32f4_ili9341.h"
#include "tm_stm32f4_fonts.h"
#include <stdio.h>
#include "Default.h"
#include <math.h>

long map(long x, long in_min ,long in_max , long out_min, long out_max);
int map_int(int x, int in_min ,int in_max , int out_min, int out_max);

void angle_PWM(void);                           // ������ ���� ���� �ӵ� 
void _EXTI_Init(void);


void Turn_Left_180 (void);
void Turn_Right_180 (void);
void Turn_Left_90 (void);
void Turn_Right_90 (void);
void Turn_Left_45 (void);
void Turn_Right_45 (void);
void Turn_Left_30 (void);
void Turn_Right_30 (void);
void Turn_Left_0(void);
void Turn_Right_0(void);


void Senser_auto_play(void);
void New_Senser_auto_play(void);
void Feedback_Rutin(void);

void DispayTitle(void);
void _GPIO_Init(void);
void LCD_lnit();
void LCD_cycle();


void USART1_Init(void);
void USART3_Init(void);
void UART4_Init(void);
void USART2_Init(void);
void UART5_Init(void);


void USART_BRR_Configuration(uint32_t USART_BaudRate);
void USART3_BRR_Configuration(uint32_t USART_BaudRate);         // USART3       ���ڱ⼾��
void UART4_BRR_Configuration(uint32_t USART_BaudRate);           // UART4       �������   TX : PC10     RX : PC11   
void USART2_BRR_Configuration(uint32_t USART_BaudRate);           // USART2       GPS   TX : PD5   RX : PD6
void UART5_BRR_Configuration(uint32_t USART_BaudRate);           // UART5       ���ڱ� ����    TX : PC12     RX : PD2 


void TIMER3_PWM_Init(void);                     // ���� ���� 
void TIMER4_PWM_Init(void);                     // ���Ͼ� 
void TIMER7_Init(void); 
void TIMER1_PWM_Init(void);                     // �귯�� 


void Linear_UP(void);
void Linear_DOWN(void);

void SerialSendChar(uint8_t c);
void SerialSendString(char *s);
void SerialSendChar_5(uint8_t c);
void SerialSendString_5(char *s);
void SerialSendChar_4(uint8_t c);
void SerialSendString_4(char *s);


uint16_t KEY_Scan(void);
void DelayMS(unsigned short wMS);
void DelayUS(unsigned short wUS);

int CompareInit(const char *ap_string1, const char *ap_string2, unsigned int a_length);


int count = 0;

int senser_play  = 0 ;
int senser_time = 0; 
int senser_flag = 0;
int senser_delay = 0;
int senser_delay_flag = 0;
int c_flag =0 ; 
int second_check = 0;
//-----------------------------

int buffer = 0;
static char yaw_data [200]= {0,};
int yaw_cnt ;
char* yaw_cut;
char* yaw_Z[16];

//-----------------------------                 // PWM ���ɸ�  200-> 10% 1600 -> 80% ������ ���� �����ؼ� ���� 40~50�� �϶� RPM�� 3200 ���� �̴� 
double d_PWM  = 0;
int Per_PWM_1 = 0;
int Per_PWM_2 = 0;
char ch3, ch4;
int PWM_1= 600;
int PWM_2 = 600;
double Z_data;
double Z_deg;
int save_pwm = 0;
int P_PWM_1, P_PWM_2;
int speed_flag;
char PWM_2_dp[20];
char PWM_1_dp[20];

int Auto_flag = 0;
int index = 1;
int delay = 1200;

int Turn_R_180 ,Turn_L_180,Turn_L_90 ,Turn_R_90, Turn_L_45, Turn_R_45,Turn_L_0, Turn_R_0, Turn_R_30,Turn_L_30 ;


//--------------------------------              // PID ���� 

double err ;                                    // 
double pid_Kp = 0.866;
double pid_Ki = 0.150;
double pid_Kd =  0.150;

double pid_Kp_term, pid_Ki_term, pid_Kd_term  ;

double I_err_dt;
double d_err ;
double pid_control;

double prev_Input;
double prev_err;

double dt=0.04;

int goal_PWM = 0; 
double ctr_PWM ; 
int Scale = 1;
int check_flag = 0;
double angle_PID(int goal_PWM);

//--------------------------------        // Ÿ�̸� �� PID ���� Ȯ�ο� ���� 
char P[20];
char I[20];
char D[20];
char scale_B[20];
char delay_B[20];


int system_time  = 0;
int S_flag = 0;         // Ÿ�̸�  ���� 
//--------------------------------      //GPS ���� 
int GPS_Point = 0 ;                     // GPS �� �����ϸ� Ÿ�̸Ӹ� �����ϴ� ���� 
int GPS_Stop = 0;                               // GPS�� 10�ʰ� ������ �ν��ϴ� ���� 
int real_check_flag = 0;                                // ��ǥ������ ������ �� 


int val= 0; 
static char rx_buf[200]={0,};  
int rx_cnt;
char *GPS_cut;
char *GNGGA[16];
int i = 0;
uint8_t  str[20]; //������ �����ϴ� str
uint8_t  str2[20]; //�浵�� �����ϴ� str2
uint8_t  str3[20]; //������ �����ϴ� str
uint8_t  str4[20]; //�浵�� �����ϴ� str2
uint8_t  str5[20]; 
uint8_t  str6[20];               //��� ��ƾ ������ �� �˷��ִ� �� 
double sum, sum2; // sum = ���� sum2 = �浵                // ���� ����  / �浵 

double c_radians_latitude  ;                   // ���� ��ġ�� ���� ���� ��
double  c_radians_longtitude ;                  // ���� ��ġ�� ���� �浵 �� 
double radians_latitude ;               // ���� ����               //������� ����
double radians_longtitude ;             // ���� �浵               // ������� �浵
double g_radians_latitude ;             // ��ǥ ���� ����            // ��ǥ���� ���� 
double g_radians_longtitude ;           // ��ǥ ���� �浵            // ��ǥ���� �浵 

double radians_space; // ���� �Ÿ�                         
double real_space_H;           //���� �Ÿ� (����)
double real_space_V;           //���� �Ÿ� (����)
double real_space_Turm;         // ó�� ��ǥ�� ������ ��ǥ���� 
double cycle_Turm;

double radians_angle;           //���� ����
double real_angle ;             // ���� ���� 

double  check_space ;                   // ������ġ�� ��������� �Ÿ� 
double check_real_space;                // ���� ��ġ�� ��������� ���� �Ÿ� 

double save_latitude[4] ;               // ����                    �ӽ÷� 4�� ����-> ������ 8 
double save_longtitude[4];              // �浵 



uint8_t Star[20];
uint8_t Star2[20];

int save_p = 0;                         // ��ǥ �浵 ���� ���� �Ķ���� �� ���� 
double error = 0.0173;  // 1.73cm ���� 
int point = 0 ;                         // ��ǥ������ ���� �ߴ��� �˱� ���� ���� 
int check_p = 0;                // ��ǥ���� �� ���� ���� ����Ʈ �� ���� 

int start_flag = 0 ; // ��� ���� �� ���۸�带 �˸� 
int turn_flag = 0;  // ȸ�� ���� ��� 
//--------------------------------
int test_int = 0;

int Brush_PWM =0;
//


//---------------------- RPM �� ���� ���� 
int rpm ;
double RPM ;
int RPM_timer_flag = 0;
int RPM_time_point = 0;
double RPM_m;
double test_m= 4.5;                  //10M ���� ����gggggg
double test_mv = 0.5;
uint8_t str_RPM[20];
int RPM_timer_test_flag ;
//----------------------  �ǵ庤 ���� ���� 
int feedback_flag ;
double save_RPM_m;

//----------------------  

int index_flag ;

int main(void)
{
  LCD_lnit();
  
  DelayMS(100);	
  _GPIO_Init(); 
  _EXTI_Init();
  
  USART1_Init();
  UART4_Init();
  USART2_Init();
  UART5_Init();
  
  
  TIMER3_PWM_Init();
  //TIMER4_PWM_Init();
  TIMER7_Init();    
  TIMER1_PWM_Init();
  
  //  �׽�Ʈ ���� �浵 // 
  save_latitude[0] = 37.340746;               // ����             
  save_longtitude[0] = 126.732760;              // �浵 
  save_latitude[1] = 37.340772;               // ����             
  save_longtitude[1] = 126.732717;              // �浵 
  save_latitude[2] = 37.340754;               // ����             
  save_longtitude[2] = 126.732700;              // �浵 
  save_latitude[3] = 37.340692;               // ����             
  save_longtitude[3] = 126.732707;              // �浵 
  // �׽�Ʈ ���� �浵 //
  
  
  
  
  // 1. ���� : 37.340772
  //    �浵 : 126.732717                                                   
  //2. ���� : 37.340746
  //    �浵 : 126.732760        
  GPIOG->ODR &= ~(1<<8);                  // �ǵ庤 
  GPIOG->ODR &= ~(1<<9);                  // �ǵ庤 
  
  GPIOG->ODR &= ~(1<<1);                     // ���� ���� CCW // ���� ���� 
  GPIOG->ODR |= (1<<2);                     //enable off
  GPIOG->ODR |= (1<<3);                     // break off 
  GPIOG->ODR |= (1<<4);                     // ���� ���� CW           // ������ ����
  GPIOG->ODR |= (1<<5);                     //enable off
  GPIOG->ODR |= (1<<6);                     // break off 
  
  
  GPIOE->ODR |= (1<<7);                   // ���Ͼ� high
  GPIOE->ODR |= (1<<8);                            // ���Ͼ� high
  GPIOE->ODR |= (7<<0); // Brush ���� OFF
  GPIOE->ODR &= ~(1<<1); // Brush ���� DIR ON 
  
  
  
  
  
  GPIOE->ODR |= (1<<0);                   //EN 
  
  
  while(1)
  {
    
    LCD_cycle();
    
    
    // ������1. ���� ���� ������ �ޱ� ���� Ŀ������ �۵��� // �ѹ� �� �������� �ʱ�ȭ ���� �ʿ䰡 �ֱ⿡ 
    // ��ƾ�� �������� ex) 1000�� ���� err �� �ʱ�ȭ  �̷�������? 
    
    //save_pwm                                                            // ���� ���� ������ ���� 
    //           SSD1306_DrawChar(51, 21, GPS_Stop%10 +0x30);
    //          SSD1306_DrawChar(51, 31, point+0x30);
    //          SSD1306_DrawChar(57, 31, save_p+0x30);
    
    
    if(Auto_flag == 1)
    {
      ctr_PWM = angle_PID(goal_PWM);
      
      
      Per_PWM_1 -=  (int)(Scale*ctr_PWM) ;
      Per_PWM_2 += (int)(Scale*ctr_PWM);
      
      if(Per_PWM_1  >=  200)             // 10%  �� �߰��� �� �ֵ��� 
        Per_PWM_1 = 200;
      else if (Per_PWM_1 <=  -200)               // 0% 
        Per_PWM_1 = -200;
      
      if(Per_PWM_2 >= 200)             // 10%
        Per_PWM_2 = 200;
      else if (Per_PWM_2 <= -200)               // 0% 
        Per_PWM_2 = -200;
      //              
      
      
      PWM_1 +=  Per_PWM_1;
      PWM_2 +=  Per_PWM_2;
      
      if(PWM_1 >= 1170)
        PWM_1 = 1170;
      else if(PWM_1 <= 770)
        PWM_1 = 770;
      if(PWM_2 >= 1500)
        PWM_2 = 1500;
      else if(PWM_2 <= 900)
        PWM_2 = 900;
      
      
      
      for(int i = 0 ; i < delay; ++i);
      
      TIM3->CCR1 = PWM_1  ; 
      TIM3->CCR2 = PWM_2 ; 
    }
    else
    {
      PWM_1 = PWM_2 = 600 ;
      TIM3->CCR1 = PWM_1  ; 
      TIM3->CCR2 = PWM_2 ;
      
    }
    
    
    if(count)                     // ���ڱ⼾�� ���� 0���� �ʱ�ȭ 
    {
      while(Z_deg >=  0.10  ||  Z_deg <=  -0.10 )
      {
        
        SerialSendChar_5('z');
        SerialSendChar_5('r');
        SerialSendChar_5('o');
        SerialSendChar_5('\n');
        
      }
      count = 0 ;
    }
    
    
    
    //-----------------------------------------//    PID_control �� // 12�ʿ� ���� 1�� �Ѿ�ϱ� �뷫5~8�� ���� �ʱ�ȭ 
    //        sprintf(str,"%5f",pid_control);
    //        SSD1306_DrawText(1, 31,str );
    //-----------------------------------------//  Ÿ�̸� �ð� ���� ����     
    // SSD1306_DrawChar(1+54, 1,S_flag+0x30 );
    //-----------------------------------------//   
    if(start_flag == 1)
    {
      
      
      //     c_radians_latitude=sum *(3.141592/180);                            // ���� ���� ����
      //  c_radians_longtitude = sum2 *(3.141592/180);                         // ���� ���� �浵 
      
      
      //                                                check_space = acos(sin(radians_latitude)*sin(c_radians_latitude) + cos(radians_latitude)*cos(c_radians_latitude)*cos(radians_longtitude-c_radians_longtitude));
      //                                                check_real_space = check_space*6366692.0724;          
      // ��� - ������ ���� �Ÿ�  /--------------------------------------------------
      cycle_Turm = real_space_Turm / real_space_V ;
      
      sprintf(str3, "%.5f", real_space_H);
      sprintf(str5, "%.5f", real_space_V);
      sprintf(str6, "%.5f", cycle_Turm);
      //sprintf(str4, "%.5f", check_real_space );
      
      //SSD1306_DrawText(1, 21, str3);                // ������ ��
      //SSD1306_DrawText(1, 31, str4);                //  ���� ��ġ �� 
      //-----------------------------------------------------
      
      start_flag = 0;
    }
    sprintf(str6, "%.5f", cycle_Turm);
    //���� 90 ��
    
    if(Turn_L_90)
    {
      Turn_Left_90();
      Auto_flag = 0;
    }
    
    //������ 90�� 
    if(Turn_R_90)
    {
      Turn_Right_90();
      Auto_flag = 0;
    }
    //���� 45��
    if(Turn_L_45)
    {
      Turn_Left_45();
      Auto_flag = 0;
    }
    //������ 45��
    if(Turn_R_45)
    {
      Turn_Right_45();
      Auto_flag = 0;
    }
    
    
    
    /*********************** ��Ʈ ��� �����̰� �ϴ� �κ� **********************/
    if(senser_play == 1 &&  (feedback_flag   != 1 || feedback_flag   != 2) )
    {
      if(cycle_Turm <= 2.5)
      {
        New_Senser_auto_play();
        if(index_flag == 1)
        {
          senser_play = 0;
          index_flag = 0;
        }
      }
      else if(cycle_Turm >2.5 && cycle_Turm  <= 4.5 )
      {
        if(index_flag <= 1)
        {
          New_Senser_auto_play();
          
        }
        else
        {
          senser_play = 0;
          index_flag = 0 ;
        }
      }
      else if(cycle_Turm >4.5 &&  cycle_Turm  <= 6.5 )
      {
        if(index_flag <= 2)
        {
          New_Senser_auto_play();
          
        }
        else
        {
          senser_play = 0;
          index_flag = 0 ;
        }      
      }
      else if(cycle_Turm >6.5 &&  cycle_Turm  <= 8.5 )
      {
        if(index_flag <= 3)
        {
          New_Senser_auto_play();
          
        }
        else
        {
          senser_play = 0;
          index_flag = 0 ;
        }      
      }
      else if(cycle_Turm >8.5 &&  cycle_Turm  <= 10.5 )
      {
        if(index_flag <= 4)
        {
          New_Senser_auto_play();
          
        }
        else
        {
          senser_play = 0;
          index_flag = 0 ;
        }      
      }
      
    }
    // �ǵ庤 �κ� 
    if(feedback_flag == 1)
    {
      RPM_timer_flag = 0;
      GPIOG->ODR |= (1<<6);                   //brk off
      GPIOG->ODR |= (1<<3);                   //brk off 
      
      GPIOG->ODR &= ~(1<<8);              // ������ ī�޶� ���� 
      
      GPIOE->ODR |= (1<<2);                    // �귯�� ENA off
      
      Brush_PWM= 1000;
      TIM1->CCR1	= Brush_PWM;		// CCR1 value
      
      senser_delay_flag = 1;
      while(senser_delay_flag == 1)  ;                // 2�� ����   
      
      
      
      GPIOG->ODR &= ~(1<<4);                  // ������ CCW
      GPIOG->ODR |= (1<<1);                    // ���� CW
      
      /*   �ӵ��� 400���� ����    */
      Auto_flag = 0;
      save_pwm = 400;
      PWM_1= save_pwm; 
      PWM_2  = save_pwm;
      TIM3->CCR2 = PWM_2;
      TIM3->CCR1	= PWM_1;        
      /*   �� ��    */
      
      senser_delay_flag = 5;
      while(senser_delay_flag == 5)  ;                //3������ 
      
      senser_delay_flag = 1;
      while(senser_delay_flag == 1)  ;                // 2�� ����   
      
      Turn_R_30= 1;
      Turn_Right_30();
      
      senser_delay_flag = 1;
      while(senser_delay_flag == 1)  ;                // 2�� ����     
      
      senser_delay_flag = 3;
      while(senser_delay_flag == 3)  ;                // 2�� ����     
      
      senser_delay_flag = 1;
      while(senser_delay_flag == 1)  ;                // 2�� ����   
      
      senser_delay_flag  = 4;
      while(senser_delay_flag == 4)  ;                // 2�� ����
      
      senser_delay_flag = 1;
      while(senser_delay_flag == 1)  ;                // 2�� ����   
      
      Turn_L_0 = 1;
      Turn_Left_0();                                                  // 0�� ��ġ
      
      senser_delay_flag = 1;
      while(senser_delay_flag == 1)  ;                // 2�� ����   
      
      senser_delay_flag = 6;
      
      while(senser_delay_flag == 6)  ;                //3�� ����              
      
      
      
      GPIOG->ODR |= (1<<8);              // ������ ī�޶� �ѱ�
      
      feedback_flag = 0 ;
      RPM_timer_flag = 1;
      RPM_m = save_RPM_m ; 
      Auto_flag = 1;
      GPIOG->ODR &= ~(1<<6);                   //brk on
      GPIOG->ODR &= ~(1<<3);
      GPIOE->ODR &= ~(1<<2);                    // �귯�� ENA on
      
    }
    else if(feedback_flag == 2)
    {
      RPM_timer_flag = 0;
      GPIOG->ODR |= (1<<6);                   //brk off
      GPIOG->ODR |= (1<<3);                   //brk off 
      
      
      GPIOG->ODR &= ~(1<<9);              // ���� ī�޶� ���� 
      
      GPIOE->ODR |= (1<<2);                    // �귯�� ENA off
      
      Brush_PWM= 1000;
      TIM1->CCR1	= Brush_PWM;		// CCR1 value
      
      senser_delay_flag = 1;
      while(senser_delay_flag == 1)  ;                // 2�� ����   
      
      
      
      GPIOG->ODR &= ~(1<<4);                  // ������ CCW
      GPIOG->ODR |= (1<<1);                    // ���� CW
      
      /*   �ӵ��� 400���� ����    */
      Auto_flag = 0;
      save_pwm = 400;
      PWM_1= save_pwm; 
      PWM_2  = save_pwm;
      TIM3->CCR2 = PWM_2;
      TIM3->CCR1	= PWM_1;        
      /*   �� ��    */
      
      senser_delay_flag = 5;
      while(senser_delay_flag == 5)  ;                //3������ 
      
      senser_delay_flag = 1;
      while(senser_delay_flag == 1)  ;                // 2�� ����   
      
      Turn_L_30= 1;
      Turn_Left_30();
      
      senser_delay_flag = 1;
      while(senser_delay_flag == 1)  ;                // 2�� ����     
      
      senser_delay_flag = 3;
      while(senser_delay_flag == 3)  ;                // 2�� ����     
      
      senser_delay_flag = 1;
      while(senser_delay_flag == 1)  ;                // 2�� ����   
      
      senser_delay_flag  = 4;
      while(senser_delay_flag == 4)  ;                // 2�� ����
      
      senser_delay_flag = 1;
      while(senser_delay_flag == 1)  ;                // 2�� ����   
      
      Turn_R_0 = 1;
      Turn_Right_0();                                                  // 0�� ��ġ
      
      senser_delay_flag = 1;
      while(senser_delay_flag == 1)  ;                // 2�� ����   
      
      senser_delay_flag = 6;
      
      while(senser_delay_flag == 6)  ;                //3�� ����              
      
      
      GPIOG->ODR |= (1<<9);              // ������ ī�޶� ���� 
      
      feedback_flag = 0 ;
      RPM_timer_flag = 1;
      RPM_m = save_RPM_m ; 
      Auto_flag = 1;
      GPIOG->ODR &= ~(1<<6);                   //brk on
      GPIOG->ODR &= ~(1<<3);
      GPIOE->ODR &= ~(1<<2);                    // �귯�� ENA on
      
      
      
    }
    
    /*********************** ��Ʈ ��� �����̰� �ϴ� �κ� **********************/
    
    
    
    
  }
}
void Feedback_Rutin()
{
  if(RPM_timer_test_flag ==0)
  {
    GPIOG->ODR |= (1<<8);                // ��ȣ �����ִ� �� 
    
    GPIOG->ODR &= ~(1<<9);                // ��ȣ �����ִ� �� 
    EXTI->IMR  |= (1<<11);  	//  EXTI8~15 ���ͷ�Ʈ mask (Interrupt Enable) ����
    EXTI->IMR  &= ~(1<<10);        
    
    
  }
  else if(RPM_timer_test_flag  == 3)
  {
    GPIOG->ODR &= ~(1<<8);
    
    GPIOG->ODR |= (1<<9);                // ��ȣ �����ִ� �� 
    EXTI->IMR  &= ~(1<<11);  	//  EXTI8~15 ���ͷ�Ʈ mask (Interrupt Enable) ����
    EXTI->IMR  |= (1<<10); 
  }
  else
  {
    GPIOG->ODR &= ~(1<<8);
    
    GPIOG->ODR &= ~(1<<9);                // ��ȣ �����ִ� �� 
    EXTI->IMR  &= ~(1<<11);  	//  EXTI8~15 ���ͷ�Ʈ mask (Interrupt Enable) ����
    EXTI->IMR  &= ~(1<<10); 
  }
}

void Senser_auto_play()
{
  
  if(senser_flag == 0 &&senser_delay_flag ==0)
  {
    GPIOE->ODR &= ~(1<<2);
    GPIOG->ODR |= (1<<4);           // ������ CW 
    GPIOG->ODR &= ~(1<<1);            // ���� CCW 
    GPIOG->ODR &= ~(1<<2);                     //enable on
    GPIOG->ODR &= ~(1<<5);  
    
  }
  else if(senser_flag == 1 &&senser_delay_flag ==0)
  {
    count = 1 ;
    
    senser_delay = 0;
    Turn_L_90 = 1;
    GPIOG->ODR |= (1<<2);                     //enable off
    GPIOG->ODR |= (1<<5);
    senser_time =0;
    Turn_Left_90 ();
    GPIOE->ODR &= ~(1<<2);            // on
    senser_flag = 2;
    senser_delay_flag = 1;
    while(senser_delay_flag == 1)
    {
      if(senser_delay_flag == 0 )
        break;
    }
    GPIOE->ODR &= ~(1<<2);     
    GPIOG->ODR |= (1<<4);           // ������ CW 
    GPIOG->ODR &= ~(1<<1);            // ���� CCW 
    
    GPIOG->ODR &= ~(1<<2);                     //enable on
    GPIOG->ODR &= ~(1<<5);  
    senser_time =0;
  }
  else if(senser_flag == 3 &&senser_delay_flag ==0)
  {
    count = 1 ;
    GPIOE->ODR &= ~(1<<2); //off
    senser_delay = 0;
    Turn_L_90 = 1;
    senser_time =0;
    Turn_Left_90 ();
    senser_flag = 4;
    
    senser_delay_flag = 1;
    while(senser_delay_flag == 1)
    {
      if(senser_delay_flag == 0 )
        break;
    }
    GPIOE->ODR &= ~(1<<2);
    
    GPIOG->ODR |= (1<<4);           // ������ CW 
    GPIOG->ODR &= ~(1<<1);            // ���� CCW 
    
    GPIOG->ODR &= ~(1<<2);                     //enable on
    GPIOG->ODR &= ~(1<<5);  
    senser_time =0;
  }
  else if(senser_flag == 5 &&senser_delay_flag ==0)
  {
    count = 1 ;
    senser_delay = 0;
    Turn_R_90 = 1;
    senser_time =0;
    Turn_Right_90 ();
    senser_flag = 6;
    
    senser_delay_flag = 1;
    while(senser_delay_flag == 1)
    {
      if(senser_delay_flag == 0 )
        break;
    }
    GPIOE->ODR &= ~(1<<0);
    GPIOG->ODR |= (1<<4);           // ������ CW 
    GPIOG->ODR &= ~(1<<1);            // ���� CCW 
    
    GPIOG->ODR &= ~(1<<2);                     //enable on
    GPIOG->ODR &= ~(1<<5);  
    senser_time =0;
    
  }
  else if(senser_flag == 7 &&senser_delay_flag ==0)
  {
    count = 1 ;
    senser_delay = 0;
    Turn_R_90 = 1;
    senser_time =0;
    Turn_Right_90 ();
    
    senser_flag = 0;
    senser_time =0;
    senser_delay_flag  = 0;
    //senser_play = 0;
    second_check++;
  }
  
  
}

void New_Senser_auto_play()
{
  if (senser_play == 1 && RPM_timer_flag == 0 && (RPM_timer_test_flag == 0|| RPM_timer_test_flag == 3)  )
  {
    
    
    Feedback_Rutin();
    
    
    GPIOE->ODR &= ~(1<<2);            // �귯�� enable on 
    GPIOG->ODR |= (1<<4);           // ������ CW 
    GPIOG->ODR &= ~(1<<1);            // ���� CCW 
    //  GPIOG->ODR &= ~(1<<2);                     //enable on
    //  GPIOG->ODR &= ~(1<<5);  
    GPIOG->ODR &= ~(1<<6);                   //brk on
    GPIOG->ODR &= ~(1<<3);
    
    RPM_timer_flag = 1;
    // �귯�� �����̰� , ���� �����̴� ��  �� �� RPM  ���� �Ÿ� �����Ͽ� ���� 
    // �� �� �Ÿ��� �Ȱ��� ��� ���� ���� 
    
  }
  else if(senser_play == 1 && RPM_timer_flag == 0  && RPM_timer_test_flag == 1)
  { 
    
    Feedback_Rutin();
    feedback_flag = 0;
    Auto_flag = 0;
    TIM3->CCR1 = 600 ;
    TIM3->CCR2 = 600 ;
    
    while(senser_delay_flag == 1);                   // 2�ʴ�� 
    
    /*       �������� ����  // ���� �� �ٽ� �ѹ��� ����  */
    senser_delay = 0;                               // �ð� 2�ʸ� ���ִ� ���� �ʱ�ȭ 
    
    Turn_L_90 = 1;
    Turn_Left_90 ();                    // 90�� ȸ�� �� ���� 0.5 ��        // 90�� ���� �ϰ� 
    
    senser_delay_flag = 1;
    while(senser_delay_flag == 1)  ;                         // 2�� ��� 
    
    senser_delay = 0;
    
    
    senser_delay_flag = 2;                                  
    while(senser_delay_flag == 2)    ;                       //���� �Ÿ���ŭ  �����ϴ� �� 
    
    // ������ ��ٸ��� �ð� 0.5 ��
    
    senser_delay = 0;
    
    senser_delay_flag = 1;
    while(senser_delay_flag == 1)   ;                        // 2�� ��� 
    
    RPM_timer_test_flag = 2;                    // ���� ������ ȸ���� �����ϴ� �κ� 
    senser_delay = 0;
    feedback_flag = 0;
    
    /*       �������� ����  // ���� �� �ٽ� �ѹ��� ����  */
    
    
    
  }
  else if(senser_play == 1 && RPM_timer_test_flag  == 2 &&  RPM_timer_flag == 0 )
  {
    Feedback_Rutin();
    feedback_flag = 0;
    //GPIOG->ODR &= ~(1<<8);
    // GPIOG->ODR &= ~(1<<9);
    
    Auto_flag = 0;
    TIM3->CCR1 = 600 ;
    TIM3->CCR2 = 600 ;
    
    Turn_L_180 = 1;
    Turn_Left_180 ();                    // 90�� ȸ�� �� ���� 0.5 ��   (180�� ȸ�� )
    
    
    senser_delay_flag = 1;
    while(senser_delay_flag == 1)       ;            // 2�� ��� 
    
    feedback_flag = 0;
    RPM_timer_test_flag = 3;       // �������� �����ִ� �� 
    
    
    
    
  }
  else if (senser_play == 1 && RPM_timer_test_flag  == 4 &&  RPM_timer_flag == 0 )
  {
    Feedback_Rutin();
    feedback_flag = 0;
    GPIOG->ODR &= ~(1<<9);
    GPIOG->ODR &= ~(1<<8);
    Auto_flag = 0;
    TIM3->CCR1 = 600 ;
    TIM3->CCR2 = 600 ;
    
    
    
    
    while(senser_delay_flag == 1)  ;                         // 2�� ��� 
    
    /*       �������� ����  // ���� �� �ٽ� �ѹ��� ����  */
    
    
    
    Turn_R_90 = 1;
    Turn_Right_90 ();                    // 90�� ȸ�� �� ���� 0.5 ��  
    
    senser_delay_flag = 1;
    while(senser_delay_flag == 1);
    
    
    senser_delay = 0;
    
    senser_delay_flag = 2;
    while(senser_delay_flag == 2);
    // ������ ��ٸ��� �ð� 0.5 �� 
    
    
    senser_delay_flag = 1;
    while(senser_delay_flag == 1)    ;       /// ���� ���� ��ŭ ���� 
    
    senser_delay = 0;
    feedback_flag = 0;
    RPM_timer_test_flag = 5;                    // ���� ������ ȸ���� �����ϴ� �κ� 
    
    /*       �������� ����  // ���� �� �ٽ� �ѹ��� ����  */
  }
  else if (senser_play == 1 && RPM_timer_test_flag  == 5 &&  RPM_timer_flag == 0 )
  {
    Feedback_Rutin();
    feedback_flag = 0;
    
    Auto_flag = 0;
    TIM3->CCR1 = 600 ;
    TIM3->CCR2 = 600 ;
    Turn_R_180 = 1;
    Turn_Right_180 ();                    // 90�� ȸ�� �� ���� 0.5 ��  
    
    senser_delay_flag = 1;
    while(senser_delay_flag == 1);
    
    
    senser_delay = 0;
    feedback_flag = 0;
    RPM_timer_test_flag = 0;                    // ���� ������ ȸ���� �����ϴ� �κ� 
    index_flag++;
    //                if(index_flag == 2)
    //                {
    //                  senser_play = 0;
    //                  index_flag = 0 ;
    //                }
  }
}

void Turn_Left_180 (void)
{
  Auto_flag = 0;
  
  if(Turn_L_180 == 1)
  {
    PWM_1 = PWM_2 = 600;
    //GPIOG->ODR &= ~(1<<2);                     //enable on
    //GPIOG->ODR &= ~(1<<5);                     //enable on
    
    GPIOG->ODR &= ~(1<<6);                   //brk on
    GPIOG->ODR &= ~(1<<3);
    while(Z_deg<=177)
    {
      
      GPIOG->ODR |= (1<<4);           // ������ CW 
      GPIOG->ODR |= (1<<1);            // ���� CW 
    }
    GPIOG->ODR |= (1<<4);           // ������ CW 
    GPIOG->ODR &= ~(1<<1);            // ���� CCW 
    // GPIOG->ODR |= (1<<5);                     //enable off
    //  GPIOG->ODR |= (1<<2);                     //enable off
    GPIOG->ODR |= (1<<6);                   //brk on
    GPIOG->ODR |= (1<<3);
    count = 1;
    Turn_L_180= 0;
    Auto_flag = 1;
  }
  
}

void Turn_Right_180 (void)
{
  Auto_flag = 0;
  if(Turn_R_180 == 1)
  {
    //GPIOG->ODR &= ~(1<<2);                     //enable on
    //GPIOG->ODR &= ~(1<<5);                     //enable on
    
    GPIOG->ODR &= ~(1<<6);                   //brk on
    GPIOG->ODR &= ~(1<<3);
    
    PWM_1 = PWM_2 = 600;
    while(Z_deg>=-177)
    {
      
      GPIOG->ODR &= ~(1<<4);           // ������ CCW 
      GPIOG->ODR &= ~(1<<1);            // ���� CCW 
    }
    GPIOG->ODR |= (1<<4);           // ������ CW 
    GPIOG->ODR &= ~(1<<1);            // ���� CCW 
    //GPIOG->ODR |= (1<<5);                     //enable off
    // GPIOG->ODR |= (1<<2);                     //enable off
    GPIOG->ODR |= (1<<6);                   //brk off
    GPIOG->ODR |= (1<<3);
    count = 1;
    Turn_R_180 = 0;
    Auto_flag = 1;
  }
  
}

void Turn_Left_90 (void)
{
  Auto_flag = 0;
  
  if(Turn_L_90 == 1)
  {
    PWM_1 = PWM_2 = 600;
    //GPIOG->ODR &= ~(1<<2);                     //enable on
    //GPIOG->ODR &= ~(1<<5);                     //enable on
    
    GPIOG->ODR &= ~(1<<6);                   //brk on
    GPIOG->ODR &= ~(1<<3);
    while(Z_deg<=87)
    {
      
      GPIOG->ODR |= (1<<4);           // ������ CW 
      GPIOG->ODR |= (1<<1);            // ���� CW 
    }
    GPIOG->ODR |= (1<<4);           // ������ CW 
    GPIOG->ODR &= ~(1<<1);            // ���� CCW 
    // GPIOG->ODR |= (1<<5);                     //enable off
    //  GPIOG->ODR |= (1<<2);                     //enable offs
    GPIOG->ODR |= (1<<6);                   //brk on
    GPIOG->ODR |= (1<<3);
    
    Turn_L_90= 0;
    
  }
  
}
void Turn_Right_90 (void)
{
  Auto_flag = 0;
  if(Turn_R_90 == 1)
  {
    //GPIOG->ODR &= ~(1<<2);                     //enable on
    //GPIOG->ODR &= ~(1<<5);                     //enable on
    
    GPIOG->ODR &= ~(1<<6);                   //brk on
    GPIOG->ODR &= ~(1<<3);
    
    PWM_1 = PWM_2 = 600;
    while(Z_deg>=-88)
    {
      
      GPIOG->ODR &= ~(1<<4);           // ������ CCW 
      GPIOG->ODR &= ~(1<<1);            // ���� CCW 
    }
    GPIOG->ODR |= (1<<4);           // ������ CW 
    GPIOG->ODR &= ~(1<<1);            // ���� CCW 
    //GPIOG->ODR |= (1<<5);                     //enable off
    // GPIOG->ODR |= (1<<2);                     //enable off
    GPIOG->ODR |= (1<<6);                   //brk off
    GPIOG->ODR |= (1<<3);
    
    Turn_R_90 = 0;
    
  }
  
}
void Turn_Left_45 (void)
{
  Auto_flag = 0;
  if(Turn_L_45 == 1)
  {
    //GPIOG->ODR &= ~(1<<2);                     //enable on
    // GPIOG->ODR &= ~(1<<5);                     //enable on
    GPIOG->ODR &= ~(1<<6);                   //brk on
    GPIOG->ODR &= ~(1<<3);      
    PWM_1 = PWM_2 = 600;
    while(Z_deg<=43)                      // ������ 10���� ���� �� 
    {
      
      GPIOG->ODR |= (1<<4);           // ������ CCW 
      GPIOG->ODR |= (1<<1);            // ���� CCW 
    }
    //   GPIOG->ODR |= (1<<5);                     //enable off
    //  GPIOG->ODR |= (1<<2);                     //enable off
    GPIOG->ODR |= (1<<6);                   //brk off
    GPIOG->ODR |= (1<<3);
    Turn_L_45 = 0;
    
    Auto_flag = 1;
  }
  
}
void Turn_Right_45 (void)
{
  Auto_flag = 0;
  if(Turn_R_45 == 1)
  {
    //  GPIOG->ODR &= ~(1<<2);                     //enable on
    //   GPIOG->ODR &= ~(1<<5);                     //enable on
    
    GPIOG->ODR &= ~(1<<6);                   //brk on
    GPIOG->ODR &= ~(1<<3);
    PWM_1 = PWM_2 = 600;
    while(Z_deg>=-43)
    {
      
      GPIOG->ODR &= ~(1<<4);           // ������ CCW 
      GPIOG->ODR &= ~(1<<1);            // ���� CCW 
    }
    //GPIOG->ODR |= (1<<5);                     //enable off
    //GPIOG->ODR |= (1<<2);                     //enable off
    GPIOG->ODR |= (1<<6);                   //brk off
    GPIOG->ODR |= (1<<3);
    
    Turn_R_45 =0;
    
  }
  
}
void Turn_Right_30 (void)
{
  Auto_flag = 0;
  if(Turn_R_30 == 1)
  {
    //  GPIOG->ODR &= ~(1<<2);                     //enable on
    //   GPIOG->ODR &= ~(1<<5);                     //enable on
    
    GPIOG->ODR &= ~(1<<6);                   //brk on
    GPIOG->ODR &= ~(1<<3);
    PWM_1 = PWM_2 = 600;
    while(Z_deg>=-28)
    {
      
      GPIOG->ODR &= ~(1<<4);           // ������ CCW 
      GPIOG->ODR &= ~(1<<1);            // ���� CCW 
    }
    //GPIOG->ODR |= (1<<5);                     //enable off
    //GPIOG->ODR |= (1<<2);                     //enable off
    GPIOG->ODR |= (1<<6);                   //brk off
    GPIOG->ODR |= (1<<3);
    
    Turn_R_30 =0;
    
  }
  
}
void Turn_Left_30(void)
{
  Auto_flag = 0;
  if(Turn_L_30 == 1)
  {
    //  GPIOG->ODR &= ~(1<<2);                     //enable on
    //   GPIOG->ODR &= ~(1<<5);                     //enable on
    
    GPIOG->ODR &= ~(1<<6);                   //brk on
    GPIOG->ODR &= ~(1<<3);
    PWM_1 = PWM_2 = 600;
    while(Z_deg<=28)
    {
      
      GPIOG->ODR |= (1<<4);           // ������ CCW 
      GPIOG->ODR |= (1<<1);            // ���� CCW 
    }
    //GPIOG->ODR |= (1<<5);                     //enable off
    //GPIOG->ODR |= (1<<2);                     //enable off
    GPIOG->ODR |= (1<<6);                   //brk off
    GPIOG->ODR |= (1<<3);
    
    Turn_L_30 =0;
    
  }
  
}

void Turn_Left_0(void)
{
  Auto_flag = 0;
  if(Turn_L_0== 1)
  {
    
    GPIOG->ODR &= ~(1<<6);                   //brk on
    GPIOG->ODR &= ~(1<<3);
    PWM_1 = PWM_2 = 600;
    while(Z_deg <= -1)
    {
      
      GPIOG->ODR |= (1<<4);           // ������ CCW 
      GPIOG->ODR |= (1<<1);            // ���� CCW 
    }
    GPIOG->ODR |= (1<<6);                   //brk off
    GPIOG->ODR |= (1<<3);
    
    Turn_L_0 =0;
    
  }
  
}

void Turn_Right_0(void)
{
  Auto_flag = 0;
  if(Turn_R_0== 1)
  {
    
    GPIOG->ODR &= ~(1<<6);                   //brk on
    GPIOG->ODR &= ~(1<<3);
    PWM_1 = PWM_2 = 600;
    while(Z_deg>= -1 )
    {
      
      GPIOG->ODR &= ~(1<<4);           // ������ CCW 
      GPIOG->ODR &= ~(1<<1);            // ���� CCW 
    }
    GPIOG->ODR |= (1<<6);                   //brk off
    GPIOG->ODR |= (1<<3);
    
    Turn_R_0 =0;
    
  }
  
}

void USART1_Init(void)
{
  // USART1 : TX(PA9)
  RCC->AHB1ENR	|= (1<<0);	// RCC_AHB1ENR GPIOA Enable
  GPIOA->MODER	|= (2<<2*9);	// GPIOB PIN9 Output Alternate function mode					
  GPIOA->OSPEEDR	|= (3<<2*9);	// GPIOB PIN9 Output speed (100MHz Very High speed)
  GPIOA->AFR[1]	|= (7<<4);	// Connect GPIOA pin9 to AF7(USART1)
  
  // USART1 : RX(PA10)
  GPIOA->MODER 	|= (2<<2*10);	// GPIOA PIN10 Output Alternate function mode
  GPIOA->OSPEEDR	|= (3<<2*10);	// GPIOA PIN10 Output speed (100MHz Very High speed
  GPIOA->AFR[1]	|= (7<<8);	// Connect GPIOA pin10 to AF7(USART1)
  
  RCC->APB2ENR	|= (1<<4);	// RCC_APB2ENR USART1 Enable
  
  USART_BRR_Configuration(115200); // USART Baud rate Configuration
  
  USART1->CR1	&= ~(1<<12);	// USART_WordLength 8 Data bit
  USART1->CR1	&= ~(1<<10);	// NO USART_Parity
  USART1->CR1	|= (1<<2);	// 0x0004, USART_Mode_RX Enable
  USART1->CR1	|= (1<<3);	// 0x0008, USART_Mode_Tx Enable
  USART1->CR2	&= ~(3<<12);	// 0b00, USART_StopBits_1
  USART1->CR3	= 0x0000;	// No HardwareFlowControl, No DMA
  
  USART1->CR1 	|= (1<<5);	// 0x0020, RXNE interrupt Enable
  NVIC->ISER[1]	|= (1<<(37-32));// Enable Interrupt USART1 (NVIC 37��)
  USART1->CR1 	|= (1<<13);	//  0x2000, USART1 Enable
}





long map(long x, long in_min ,long in_max , long out_min, long out_max)
{
  return (x-in_min)*(out_max-out_min)/(in_max - in_min)+out_min;
  
}
int map_int(int x, int in_min ,int in_max , int out_min, int out_max)
{
  return (x-in_min)*(out_max-out_min)/(in_max - in_min)+out_min;
  
}
//speed_index
double angle_PID(int goal_PWM)                  // ��� �ʱ�ȭ �ص� ���� x 
{
  if(Z_deg >=  1.0  || Z_deg <= -1.0  )
  {
    err = goal_PWM -(Z_deg);
    
    pid_Kp_term = err* pid_Kp;    // err * kp
    
    I_err_dt = err*dt;                    // ���� 
    pid_Ki_term +=  I_err_dt*pid_Ki;              // i * ki
    
    prev_err = Z_deg;
    
    d_err = Z_deg -prev_err;
    
    pid_Kd_term = -pid_Kd*(d_err/dt);
    
    if( Z_deg <= -1.0 )
    {
      
      speed_flag += 5;
      if(speed_flag >200)
        speed_flag =200;
      
      TM_ILI9341_Putc(210, 300, '+' ,&TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
    }
    else if (Z_deg >= 1.0)
    {
      speed_flag -= 5;
      if(speed_flag <-200)
        speed_flag  = -200;
      
      TM_ILI9341_Putc(210, 300, '-' ,&TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
    }
    pid_control = pid_Kp_term+pid_Ki_term+pid_Kd_term;
    
  }
  else 
  {
    pid_control = 0;
    Per_PWM_1 = Per_PWM_2 = 0;
    save_pwm = 950;                                   //  960 ���� 
    PWM_1 =save_pwm-speed_flag;                                                         // ������ �� 950 // 340 
    PWM_2 = save_pwm+280+(speed_flag) ;            //118650 ���� ���� 
    TIM3->CCR1 = PWM_1 ;
    TIM3->CCR2 = PWM_2 ;
  }
  /*
  else if (Z_data <=0.2 && Z_data >= -0.2)
  {
  pid_control = 0;
  Per_PWM_1 = Per_PWM_2 = 0;
  save_pwm = 960;
  PWM_1 =save_pwm;
  PWM_2 = save_pwm+220 ;
  TIM3->CCR1 = PWM_1 ;
  TIM3->CCR2 = PWM_2 ;
}
  */
  return pid_control;
}

void UART5_IRQHandler(void)	                        // Z �� ���� 
{       
  if ( (UART5->SR & USART_SR_RXNE) ) 	// USART_SR_RXNE=(1<<5) 
  {
    char ch3;
    ch3 = (uint16_t)(UART5->DR & (uint16_t)0x01FF);	// ���ŵ� ���� ����
    
    
    if(ch3 == ' ' && index == 1)
    {
      
      yaw_data[yaw_cnt++] = ',';
      yaw_data[yaw_cnt++] = ch3;
      index = 2;
    }
    else if(ch3 == ' ' && index == 2)
    {
      if(yaw_cnt < 180)
        yaw_data[yaw_cnt++] = ch3;
      
    }   
    else if(ch3 == '!')                    // ���� ���̴� �� . 
    {
      ch3 = 0;
      for(int i = 0 ; i <yaw_cnt; i++)
      {
        yaw_data [i] = 0;
      }
      yaw_cnt = 0;
    }
    else if(ch3 == 0x0D)
    {
      yaw_data[yaw_cnt++] = 0;
    }
    else if(ch3 == 0x0A)
    {
      yaw_data[yaw_cnt++] = 0;
      yaw_data[yaw_cnt++] = ',';
      
      if(CompareInit(", ",yaw_data,2)==0)
      {
        yaw_cut = strtok(yaw_data, ",");   
        while(yaw_cut  != NULL)
        {
          for(int i = 0; i < 16; i++)
          {
            yaw_Z[i] = yaw_cut;
            yaw_cut = strtok(NULL, ",");  
          }
        }
        
      }
      
      if(yaw_Z[5][0] == 0x2D)
      {
        yaw_Z[5][0] = ' ';
      }   
      
      Z_data = atof(yaw_Z[5]);
      
      if(yaw_Z[5][0] == ' ')
      {
        Z_deg = -Z_data;
      }
      else 
      {
        Z_deg = Z_data;
      }
      
      
      index = 1;
      yaw_cnt = 0 ;  
    }
    
    else
    {
      if( yaw_data[yaw_cnt-1] ==  ' ')
      {
        yaw_data[yaw_cnt++] = ',';
      }
      yaw_data[yaw_cnt++] = ch3;
      index = 1;
      
      
    }  
    
    
    //SerialSendChar(ch);
    
  }    
  
} 
// DR �� ������ SR.RXNE bit(flag bit)�� clear �ȴ�. �� clear �� �ʿ���� 
void UART4_IRQHandler(void)	                        // ������� 
{       
  if ( (UART4->SR & USART_SR_RXNE) ) 	// USART_SR_RXNE=(1<<5) 
  {
    ch4 = (uint16_t)(UART4->DR & (uint16_t)0x01FF);	// ���ŵ� ���� ����
    //  ������ ��!!! 
    // �׻� Enable �� ���� ����! �׷��� ���� �ϸ鼭 ���� 
    // Break �� ���� ���� �� �극��ũ�� �Ǽ� ������ ��� ���� 
    // ����Ű �߰� �Ǹ� �װſ� �°� �߰� ����   
    if(ch4== '1') //           P���� UP         
    {    
      pid_Kp += 0.001; 
      
      GPIOG->ODR &= ~(1<<8);                      // ī�޶� Ű�� ���� �ſ���. 
      GPIOG->ODR &= ~(1<<9);  
      RPM_timer_flag = 1;
      
    }
    else if(ch4 =='2') //               P ���� DOWN
    {         
      pid_Kp -= 0.001;  
      GPIOG->ODR |= (1<<8);
      GPIOG->ODR |= (1<<9);  
      
      RPM_timer_flag = 0;
    }
    else if(ch4 =='3')    //            I ���� UP         // �׽�Ʈ ���� 
    {          
      // pid_Ki += 0.001; 
      EXTI->IMR  |= (1<<14);  	//  EXTI8~15 ���ͷ�Ʈ mask (Interrupt Enable) ����
      EXTI->IMR  |= (1<<15);
    }
    else if(ch4 =='4')            //    I ���� DOWN                         // �׽�Ʈ ���� 
    {      
      //  pid_Ki -= 0.001;
      EXTI->IMR  &= ~(1<<14);  	//  EXTI8~15 ���ͷ�Ʈ mask (Interrupt Enable) ����
      EXTI->IMR  &= ~(1<<15);
    }
    else if(ch4 == '5')  //             D���� UP                          // �׽�Ʈ ���� 
    {
      // pid_Kd +=  0.001;
      
      RPM_timer_flag = 1;               //RPM timer �׽�Ʈ 
    }           
    else if(ch4 == '6')    //           D ���� DOWN                  // �׽�Ʈ ����      
    {
      //  pid_Kd -=  0.001;
      
      RPM_timer_flag = 0;
      RPM_m = 0;
    }
    else if(ch4 == '7')    //           Scale UP                        // �׽�Ʈ ���� 
    {
      //   Scale += 1;
      senser_play = 1;
      
    }
    else if(ch4 == '8')    //           Scale DOWN              // �׽�Ʈ ���� 
    {
      //  Scale -= 1;
      senser_play = 0;
      senser_flag = 0;
      senser_time =0;
    }
    else if(ch4 =='O')                  // ���̷� ������ ��� ON                // �ڵ� ���� ON 
    {
      Auto_flag  = 1;
    }
    else if(ch4 =='T')                  // ���̷� ���� ��� OFF                // �ڵ� ���� OFF
    {
      Auto_flag  = 0 ;
      save_pwm = 600;
      PWM_1= save_pwm; 
      PWM_2  = save_pwm;
      TIM3->CCR2 = PWM_2;
      TIM3->CCR1	= PWM_1;	
      err = pid_Kp_term = pid_Ki_term = pid_Kd_term = pid_control = d_err = I_err_dt = prev_err  = 0;
      goal_PWM  = 0 ;
      S_flag = 0;
    }
    else if(ch4 == 's')                 // �ʱ�ȭ
    {
      NVIC_SystemReset();  
    }
    else if(ch4 == 'Z')                          // GPS �Է� ���� �� ��� ���� �ϰ� �Ǵ� Ű 
    {
      /*
      1.ENA ���� ON 
      2. BRK ���� OFF 
      3. ���� -> �ڵ����� ���� 
      4. �귯�� ENA, BRK ON 
      5. �귯�� �ӵ� 50% ���߱� 
      6.GPS �� ������ ǥ�� �ϱ� .
      7. û�ҹ�ư ���� �� ���� 
      8. �ѹ� ���ڱ� ���� �ʱ�ȭ 
      9. ī�޶� PG8 �� ON / ���ͷ�Ʈ PC14 ����ŷ ON 
      */        
      count = 1;                        // ���ڱ� ���� �ʱ�ȭ 
      
      GPIOG->ODR |= (1<<4);             // ������ CW 
      GPIOG->ODR &= ~(1<<1);            // ���� CCW 
      
      GPIOG->ODR &= ~(1<<5);            // ������ ENA ON 
      GPIOG->ODR &= ~(1<<2);          // ���� ENA ON
      
      GPIOG->ODR |= (1<<6);               //������ BRK OFF 
      GPIOG->ODR |= (1<<3);         // ���� BRK OFF 
      
      GPIOE->ODR &= ~(1<<0);                    // �귯�� ENA on
      GPIOE->ODR &= ~(1<<2);                    //�귯�� BRK on 
      
      Brush_PWM= 1000;                          // �귯�� �ӵ� 50% 
      TIM1->CCR1	= Brush_PWM;		// CCR1 value
      
      Auto_flag  = 1;                  // ���� -> �ڵ� 
      start_flag = 1;                   // GPS �� ���� ���� ǥ�� 
      
      // GPIOG->ODR |= (1<<8);
      // EXTI->IMR  |= (1<<14);  	//  EXTI8~15 ���ͷ�Ʈ mask (Interrupt Enable) ����            ī�޶󿡼� ���� �����͸� ���� �ǵ庤 ������ ���� 
      
      senser_play = 1;
      
      
    }
    
    //-------------- ������ ���� --------------------//
    
    else if(ch4 == 'A') //  ���� ���� CCW
    {           
      GPIOG->ODR &= ~(1<<4);
      
    }
    else if(ch4 =='B')          // ���� ���� CW 
    {    
      GPIOG->ODR |= (1<<4);
    }
    
    else if(ch4 =='C')                    // Enable ON
    {          
      GPIOG->ODR &= ~(1<<5);
    }
    else if(ch4 =='D')                // Enable Off
    {       
      GPIOG->ODR |= (1<<5);
    }
    else if(ch4 =='E')            // break ON
    {
      GPIOG->ODR &= ~(1<<6);     
    }
    else if(ch4 =='F')            // Break Off
    {
      GPIOG->ODR |= (1<<6);
    }
    else if(ch4 =='G')            // Speed UP           // 1% ��
    {
      PWM_2  += 5;
      TIM3->CCR2	= PWM_2;	                 //
      
    }
    else if(ch4 =='H')          // Speed DOWN
    {
      PWM_2  -= 5;
      TIM3->CCR2	= PWM_2;	                 //
      
    }
    //----------------    ���� ���� --------------------------//  
    else if(ch4 == 'a') //  ���� ���� CCW
    {          
      GPIOG->ODR &= ~(1<<1);
    }
    else if(ch4 =='b')          // ���� ���� CW 
    {    
      GPIOG->ODR |= (1<<1);
    }
    
    else if(ch4 =='c')                    // Enable ON
    {          
      GPIOG->ODR &= ~(1<<2);
    }
    else if(ch4 =='d')                // Enable Off
    {       
      GPIOG->ODR |= (1<<2);
    }
    else if(ch4 =='e')            // break ON
    {
      GPIOG->ODR &= ~(1<<3);
    }
    else if(ch4 =='f')            // Break Off
    {
      GPIOG->ODR |= (1<<3);
    }
    else if(ch4 =='g')            // Speed UP           // 1% ��
    {
      PWM_1  += 5;
      save_pwm = PWM_1;
      TIM3->CCR1	= PWM_1;	                 //
      
    }
    else if(ch4 =='h')          // Speed DOWN
    {
      PWM_1  -= 5;
      TIM3->CCR1	= PWM_1;	                 //
    }
    
    else if (ch4 == 'r')                        // ��ȸ��   �Ѵ� CCW 
    {
      GPIOG->ODR &= ~(1<<4);           // ������ CCW 
      GPIOG->ODR &= ~(1<<1);            // ���� CCW 
    }
    else if (ch4 == 'l')                        // ��ȸ��  ���� CW  ������ CW 
    {
      GPIOG->ODR |= (1<<4);           // ������ CW 
      GPIOG->ODR |= (1<<1);            // ���� CW 
    }
    
    else if(ch4 =='R')          // err �� �ʱ�ȭ 
    {
      err = pid_Kp_term = pid_Ki_term = pid_Kd_term = pid_control = d_err = I_err_dt = prev_err  = 0;
      goal_PWM  = 0 ;
      speed_flag = 0;
      count = 1;
    }
    else if(ch4 =='M')          // ������ UP
    {
      delay += 100;
    }
    else if(ch4 =='N')          // ������ DOWN
    {
      delay -= 100;
    }
    else if(ch4 =='S')          
    {
      
      GPS_Point = 1;
      
    }
    else if(ch4 == 'Y')
    {
      point=0 ;
      start_flag = 1; 
    }
    /*****************************************ȸ��******************************************************/
    else if(ch4 == 't')                 //���� 90�� 
    {
      
      Turn_L_90= 1;
      
    }
    else if(ch4 == 'u')                 // ������ 90�� 
    {
      
      Turn_R_90 = 1;
    }
    else if(ch4 == 'v')                 // ���� 45�� 
    {
      
      Turn_L_45 = 1;
    }
    else if(ch4 == 'w')                 // ������ 45�� 
    {
      
      Turn_R_45 = 1;
    }
    
    
    
    /*****************************************Brush******************************************************/
    /*****************************************Brush******************************************************/
    /*****************************************Brush******************************************************/
    /*****************************************Brush******************************************************/
    else if(ch4 == 'i')
    {
      Brush_PWM = 400;
      TIM1->CCR1	= Brush_PWM;		// CCR1 value
    }
    else if(ch4 == 'j')
    {
      Brush_PWM= 1000;
      TIM1->CCR1	= Brush_PWM;		// CCR1 value
    }
    else if(ch4 == 'k')
    {
      Brush_PWM-=20;
      TIM1->CCR1	= Brush_PWM;		// CCR1 value
    }
    else if(ch4 == 'n')
    {
      Brush_PWM+=20;
      TIM1->CCR1	= Brush_PWM;		// CCR1 value
    }
    else if(ch4 == 'm') //ENA ON
    {
      GPIOE->ODR &= ~(1<<2);                    //ena
      
    }
    else if(ch4 == 'o') // ENA OFF
    {
      GPIOE->ODR |= (1<<2);                             //ENA
    }
    else if(ch4 == 'p') // BRK ON
    {
      GPIOE->ODR &= ~(1<<0);                    //BRK
    }
    else if(ch4 == 'q') // BRK OFF
    {
      GPIOE->ODR |= (1<<0);                     //BRK
    }
    
    /*****************************************Linar Actuater******************************************************/
    /*****************************************Linar Actuater******************************************************/
    /*****************************************Linar Actuater******************************************************/
    /*****************************************Linar Actuater******************************************************/
    
    else if(ch4 == 'I') // Linar DOWN
    {
      Linear_DOWN();
    }
    else if(ch4 == 'J') // Linar Up
    {
      Linear_UP();
    }
    else if(ch4 == 'K') // Linear stop() 
    {
      GPIOE->ODR &= ~ (15<<3);
    }
    
    
    sprintf(P,"%.3f",pid_Kp);
    sprintf(I,"%.3f",pid_Ki);
    sprintf(D,"%.3f",pid_Kd);
    //sprintf(scale_B,"%d",(int)Scale);
    //sprintf(delay_B,"%d",(int)delay);
    
    
    SerialSendString_4(P);
    SerialSendChar_4(' ');
    SerialSendString_4(I);
    SerialSendChar_4(' ');
    SerialSendString_4(D);
    SerialSendChar_4('\n');
    //SerialSendString_4(scale_B);
    //SerialSendString_4(PWM_1_dp);
    //SerialSendChar_4(' ');
    //SerialSendString_4(delay_B);
    //   SerialSendString_4(PWM_2_dp);
    SerialSendChar_4('\n');
    SerialSendChar_4('\n');
  }
  
}

//void USART1_IRQHandler(void)	
//{       
//	if ( (USART1->SR & USART_SR_RXNE) ) 	// USART_SR_RXNE=(1<<5) 
//	{
//		char ch;
//		ch = (uint16_t)(USART1->DR & (uint16_t)0x01FF);	// ���ŵ� ���� ����
//
//	} 
//        // DR �� ������ SR.RXNE bit(flag bit)�� clear �ȴ�. �� clear �� �ʿ���� 
//}



void USART2_IRQHandler(void)	// GPS                   ��ǥ ���� , ��ǥ �� �޾ƿ��� ��� �� ���� .
{       
  if ( (USART2->SR & USART_SR_RXNE) ) 	// USART_SR_RXNE=(1<<5) 
  {
    char ch;
    ch = (uint16_t)(USART2->DR & (uint16_t)0x01FF);	// ���ŵ� ���� ����
    //   SerialSendChar(ch); 
    
    if(ch=='$')
    { 
      rx_cnt=0; 
      rx_buf[rx_cnt++] = ch; 
    }
    
    
    else if(ch == ',')
    {
      if(rx_buf[rx_cnt-1] == ',') rx_buf[rx_cnt++] = '0'; 
      rx_buf[rx_cnt++] = ch;
    }
    
    
    else if(ch == '*')
    {
      if(rx_buf[rx_cnt-1]==',') rx_buf[rx_cnt++] = '0';
      rx_buf[rx_cnt++] = ',';
      rx_buf[rx_cnt++] = ch;
    }
    
    else if(ch == 0x0D)
    {
      rx_buf[rx_cnt++] = 0; //null
    }
    
    
    else if(ch==0x0A)
    {
      
      rx_buf[rx_cnt++] = 0; //null
      
      if(CompareInit("$GNGGA",rx_buf,6)==0)
      {
        
        GPS_cut = strtok(rx_buf, ",");   
        
        while (GPS_cut != NULL)   
        {
          for(i = 0; i < 16; i++)
          {
            GNGGA[i] = GPS_cut;
            GPS_cut = strtok(NULL, ",");  
          }
          
          
          double latitude = atof(GNGGA[2]); //�����ҷ���
          
          int latitude1 = (int)latitude / 100; // ���� �պκ�  ���������� �߳���
          
          latitude = latitude * 100000; //
          double latitude2 = ((int)latitude % 10000000)  ; // ���� �޺κ�
          latitude2 = latitude2 / 100000; 
          latitude2 = latitude2 / 60;
          
          sum = latitude1 + latitude2;
          
          //sprintf(str, "%d", latitude1 );
          sprintf(str, "%.8f", sum);
          double longtitude = atof(GNGGA[4]); //�浵�ҷ���
          
          int longtitude1 = (int)longtitude / 100; // �浵 �պκ�  ���������� �߳���
          
          longtitude = longtitude * 100000; //
          double longtitude2 = ((int)longtitude % 10000000)  ; // �浵 �޺κ�
          longtitude2 = longtitude2 / 100000; 
          longtitude2 = longtitude2 / 60;
          
          sum2 = longtitude1 + longtitude2;
          
          //sprintf(str3, "%d", longtitude1);
          sprintf(str2, "%.8f", sum2);
          
          //                                        SSD1306_DrawText(1, 1, str);  // ���� 
          //                                        SSD1306_DrawText(1, 11, str2);        // �浵 
          //                                        SSD1306_DrawText(1, 21, str3);
          //                                        SSD1306_DrawText(1, 31, str4);
          
          
          
          
          
        }
      }
    }
    
    else 
    {
      if(rx_cnt<180) rx_buf[rx_cnt++] = ch;
      
    }
    
    
    
    
  } 
  // DR �� ������ SR.RXNE bit(flag bit)�� clear �ȴ�. �� clear �� �ʿ���� 
}



int CompareInit(const char *ap_string1, const char *ap_string2, unsigned int a_length)
{
  unsigned int i;
  for(i = 0; i < a_length-1; i++)
  {
    if(*ap_string1 != *ap_string2) break;
    if(*ap_string1 == 0 ) break;
    
    ap_string1++;
    ap_string2++;
    
  }
  
  if(*ap_string1 == *ap_string2) return 0;
  else if(*ap_string1 > *ap_string2) return 1;
  
  
  return -1;
  
  
}





void _GPIO_Init(void)
{
  // LED (GPIO G) ����
  RCC->AHB1ENR	|=  0x00000040;	// RCC_AHB1ENR : GPIOG(bit#6) Enable							
  GPIOG->MODER 	|=  0x00055555;	// GPIOG 0~7  8,9: Output mode (0b01)		
  
  GPIOG->OTYPER	&= ~0x00FF;	// GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
  
  RCC->AHB1ENR	|=  0x00000010;	// RCC_AHB1ENR : GPIOE(bit#1) Enable							
  GPIOE->MODER 	|=  0x00515555;	// GPIOG 0~8 ,10~11: Output mode (0b01)						
  
  GPIOE->OTYPER	&= ~0xF00FFF;	// GPIOG 0~7 : Push-pull  (GP8~15:reset state)	
  // GPIOC->OSPEEDR	|= (3<<2*10);	// GPIOC PIN10 Output speed (100MHz Very High speed)
}
void UART4_Init(void)   // ��� 
{
  // UART4 : TX(PC10)
  RCC->AHB1ENR	|= (1<<2);	// RCC_AHB1ENR GPIOC Enable
  GPIOC->MODER	|= (2<<2*10);	// GPIOC PIN10 Output Alternate function mode					
  GPIOC->OSPEEDR	|= (3<<2*10);	// GPIOC PIN10 Output speed (100MHz Very High speed)
  GPIOC->AFR[1]	|= (8<<8);	// Connect GPIOC pin10 to AF8(UART4)
  
  // UART4 : RX(PC11)
  GPIOC->MODER 	|= (2<<2*11);	// GPIOC PIN11 Output Alternate function mode
  GPIOC->OSPEEDR	|= (3<<2*11);	// GPIOC PIN11 Output speed (100MHz Very High speed
  GPIOC->AFR[1]	|= (8<<12);	// Connect GPIOC pin11 to AF8(UART4)
  
  RCC->APB1ENR	|= (1<<19);	// RCC_APB1ENR UART4 Enable
  
  UART4_BRR_Configuration(9600); // USART Baud rate Configuration
  
  UART4->CR1	&= ~(1<<12);	// USART_WordLength 8 Data bit
  UART4->CR1	&= ~(1<<10);	// NO USART_Parity
  UART4->CR1	|= (1<<2);	// 0x0004, USART_Mode_RX Enable
  UART4->CR1	|= (1<<3);	// 0x0008, USART_Mode_Tx Enable
  UART4->CR2	&= ~(3<<12);	// 0b00, USART_StopBits_1
  UART4->CR3	= 0x0000;	// No HardwareFlowControl, No DMA
  
  UART4->CR1 	|= (1<<5);	// 0x0020, RXNE interrupt Enable
  NVIC->ISER[1]	|= (1<<(52-32));// Enable Interrupt UART4 (NVIC 52��)
  UART4->CR1 	|= (1<<13);	//  0x2000, UART4 Enable
}

void UART5_Init(void) // ���ڱ� 
{
  
  NVIC_SetPriority(TIM7_IRQn ,15 );
  // UART5: TX(PC12)
  RCC->AHB1ENR   |= (1<<2);   // RCC_AHB1ENR GPIOC Enable
  GPIOC->MODER   |= (2<<2*12);   // GPIOC PIN12 Output Alternate function mode               
  GPIOC->OSPEEDR   |= (3<<2*12);   // GPIOC PIN12 Output speed (100MHz Very High speed)
  GPIOC->AFR[1]   |= (8<<16);   // Connect PC12 to AF8(UART5)
  
  // UART5 : RX(PD2)
  RCC->AHB1ENR   |= (1<<3);   // RCC_AHB1ENR GPIOD Enable
  GPIOD->MODER    |= (2<<2*2);   // GPIOD PIN10 Output Alternate function mode
  GPIOD->OSPEEDR   |= (3<<2*2);   // GPIOD PIN10 Output speed (100MHz Very High speed
  GPIOD->AFR[0]   |= (8<<8);   // Connect PD2 to AF8(UART5)
  
  RCC->APB1ENR   |= (1<<20);   // RCC_APB1ENR UART5 Enable
  
  UART5_BRR_Configuration(115200); // USART Baud rate Configuration
  
  UART5->CR1   &= ~(1<<12);   // USART_WordLength 8 Data bit
  UART5->CR1   &= ~(1<<10);   // PCE: NO USART_Parity
  
  UART5->CR1   &= ~(1<<12);   // USART_WordLength 8 Data bit
  
  UART5->CR1   |= (1<<2);   // 0x0004, USART_Mode_RX Enable
  UART5->CR1   |= (1<<3);   // 0x0008, USART_Mode_Tx Enable
  UART5->CR2   &= ~(3<<12);   // 0b00, USART_StopBits_1
  UART5->CR3   = 0x0000;   // No HardwareFlowControl, No DMA
  
  UART5->CR1    |= (1<<5);   // 0x0020, RXNE interrupt Enable
  NVIC->ISER[1]   |= (1<<(53-32));// Enable Interrupt UART5 (NVIC 53��)
  UART5->CR1    |= (1<<13);   //  0x2000, UART5 Enable
  
}


void USART2_Init(void)          // GPS 
{
  
  NVIC_SetPriority(USART2_IRQn,14 );
  // UART5: TX(PD5)
  RCC->AHB1ENR	|= (1<<3);	// RCC_AHB1ENR GPIOD Enable
  GPIOD->MODER	|= (2<<2*5);	// GPIOC PIN12 Output Alternate function mode					
  GPIOD->OSPEEDR	|= (3<<2*5);	// GPIOC PIN12 Output speed (100MHz Very High speed)
  GPIOD->AFR[1]	|= (7<<4*5);	// Connect PC12 to AF8(UART5)
  
  // UART5 : RX(PD6)
  RCC->AHB1ENR	|= (1<<3);	// RCC_AHB1ENR GPIOD Enable
  GPIOD->MODER 	|= (2<<2*6);	// GPIOD PIN10 Output Alternate function mode
  GPIOD->OSPEEDR	|= (3<<2*6);	// GPIOD PIN10 Output speed (100MHz Very High speed
  GPIOD->AFR[0]	|= (7<<4*6);	// Connect PD2 to AF8(UART5)
  
  RCC->APB1ENR	|= (1<<17);	// RCC_APB1ENR UART5 Enable
  
  USART2_BRR_Configuration(38400); // USART Baud rate Configuration
  
  USART2->CR1	&= ~(1<<12);	// USART_WordLength 8 Data bit
  USART2->CR1	&= ~(1<<10);	// PCE: NO USART_Parity
  
  USART2->CR1	&= ~(1<<12);	// USART_WordLength 8 Data  
  
  USART2->CR1	|= (1<<2);	// 0x0004, USART_Mode_RX Enable
  USART2->CR1	|= (1<<3);	// 0x0008, USART_Mode_Tx Enable
  USART2->CR2	&= ~(3<<12);	// 0b00, USART_StopBits_1
  USART2->CR3	= 0x0000;	// No HardwareFlowControl, No DMA
  
  USART2->CR1 	|= (1<<5);	// 0x0020, RXNE interrupt Enable
  NVIC->ISER[1]	|= (1<<(38-32));// Enable Interrupt UART5 (NVIC 53��)
  USART2->CR1 	|= (1<<13);	//  0x2000, UART5 Enable
}



void SerialSendChar(uint8_t Ch) // 1���� ������ �Լ�
{
  while((UART4->SR & USART_SR_TXE) == RESET); // USART_SR_TXE=(1<<7), �۽� ������ ���±��� ���
  
  USART1->DR = ((int16)Ch & 0x1FF)  ;	// ���� (�ִ� 9bit �̹Ƿ� 0x01FF�� masking)
}

void SerialSendString(char *str) // �������� ������ �Լ�
{
  while (*str != '\0') // ���Ṯ�ڰ� ������ ������ ����, ���Ṯ�ڰ� �����Ŀ��� ������ �޸� ���� �߻����ɼ� ����.
  {
    SerialSendChar(*str);	// �����Ͱ� ����Ű�� ���� �����͸� �۽�
    str++; 			// ������ ��ġ ����
  }
}
void SerialSendChar_5(uint8_t Ch) // 1���� ������ �Լ�
{
  while((UART5->SR & USART_SR_TXE) == RESET); // USART_SR_TXE=(1<<7), �۽� ������ ���±��� ���
  
  UART5->DR = ((int16)Ch & 0x1FF)  ;	// ���� (�ִ� 9bit �̹Ƿ� 0x01FF�� masking)
}

void SerialSendString_5(char *str) // �������� ������ �Լ�
{
  while (*str != '\0') // ���Ṯ�ڰ� ������ ������ ����, ���Ṯ�ڰ� �����Ŀ��� ������ �޸� ���� �߻����ɼ� ����.
  {
    SerialSendChar_5(*str);	// �����Ͱ� ����Ű�� ���� �����͸� �۽�
    str++; 			// ������ ��ġ ����
  }
}
void SerialSendChar_4(uint8_t Ch) // 1���� ������ �Լ�
{
  while((UART4->SR & USART_SR_TXE) == RESET); // USART_SR_TXE=(1<<7), �۽� ������ ���±��� ���
  
  UART4->DR = (Ch & 0x01FF);	// ���� (�ִ� 9bit �̹Ƿ� 0x01FF�� masking)
}


void SerialSendString_4(char *str) // �������� ������ �Լ�
{
  while (*str != '\0') // ���Ṯ�ڰ� ������ ������ ����, ���Ṯ�ڰ� �����Ŀ��� ������ �޸� ���� �߻����ɼ� ����.
  {
    SerialSendChar_4(*str);	// �����Ͱ� ����Ű�� ���� �����͸� �۽�
    str++; 			// ������ ��ġ ����
  }
}


// Baud rate  
void USART_BRR_Configuration(uint32_t USART_BaudRate)
{ 
  uint32_t tmpreg = 0x00;
  uint32_t APB2clock = 84000000;	//PCLK2_Frequency
  uint32_t integerdivider = 0x00;
  uint32_t fractionaldivider = 0x00;
  
  if ((USART1->CR1 & USART_CR1_OVER8) != 0) // USART_CR1_OVER8=(1<<15)
  {                                         // USART1->CR1.OVER8 = 1 (8 oversampling)
    // Computing 'Integer part' when the oversampling mode is 8 Samples 
    integerdivider = ((25 * APB2clock) / (2 * USART_BaudRate));    
  }
  else  // USART1->CR1.OVER8 = 0 (16 oversampling)
  {	// Computing 'Integer part' when the oversampling mode is 16 Samples 
    integerdivider = ((25 * APB2clock) / (4 * USART_BaudRate));    
  }
  tmpreg = (integerdivider / 100) << 4;
  
  // Determine the fractional part 
  fractionaldivider = integerdivider - (100 * (tmpreg >> 4));
  
  // Implement the fractional part in the register 
  if ((USART1->CR1 & USART_CR1_OVER8) != 0)	// 8 oversampling
  {
    tmpreg |= (((fractionaldivider * 8) + 50) / 100) & (0x07);
  }
  else 						// 16 oversampling
  {
    tmpreg |= (((fractionaldivider * 16) + 50) / 100) & (0x0F);
  }
  
  // Write to USART BRR register
  USART1->BRR = (uint16_t)tmpreg;
}

void USART3_BRR_Configuration(uint32_t USART_BaudRate)
{ 
  uint32_t tmpreg = 0x00;
  uint32_t APB1clock = 42000000;	//PCLK1_Frequency
  uint32_t integerdivider = 0x00;
  uint32_t fractionaldivider = 0x00;
  
  // Determine the integer part 
  if ((USART3->CR1 & USART_CR1_OVER8) != 0) // USART_CR1_OVER8=(1<<15)
  {                                         // UART5->CR1.OVER8 = 1 (8 oversampling)
    // Computing 'Integer part' when the oversampling mode is 8 Samples 
    integerdivider = ((25 * APB1clock) / (2 * USART_BaudRate));    
  }
  else  // UART5->CR1.OVER8 = 0 (16 oversampling)
  {	// Computing 'Integer part' when the oversampling mode is 16 Samples 
    integerdivider = ((25 * APB1clock) / (4 * USART_BaudRate));    
  }
  tmpreg = (integerdivider / 100) << 4;
  
  // Determine the fractional part 
  fractionaldivider = integerdivider - (100 * (tmpreg >> 4));
  
  // Implement the fractional part in the register 
  if ((USART3->CR1 & USART_CR1_OVER8) != 0)	// 8 oversampling
  {
    tmpreg |= (((fractionaldivider * 8) + 50) / 100) & (0x07);
  }
  else 						// 16 oversampling
  {
    tmpreg |= (((fractionaldivider * 16) + 50) / 100) & (0x0F);
  }
  
  // Write to USART BRR register
  USART3->BRR = (uint16_t)tmpreg;
  
}

void UART5_BRR_Configuration(uint32_t USART_BaudRate)
{ 
  uint32_t tmpreg = 0x00;
  uint32_t APB1clock = 42000000;	//PCLK1_Frequency
  uint32_t integerdivider = 0x00;
  uint32_t fractionaldivider = 0x00;
  
  // Determine the integer part 
  if ((UART5->CR1 & USART_CR1_OVER8) != 0) // USART_CR1_OVER8=(1<<15)
  {                                         // UART5->CR1.OVER8 = 1 (8 oversampling)
    // Computing 'Integer part' when the oversampling mode is 8 Samples 
    integerdivider = ((25 * APB1clock) / (2 * USART_BaudRate));    
  }
  else  // UART5->CR1.OVER8 = 0 (16 oversampling)
  {	// Computing 'Integer part' when the oversampling mode is 16 Samples 
    integerdivider = ((25 * APB1clock) / (4 * USART_BaudRate));    
  }
  tmpreg = (integerdivider / 100) << 4;
  
  // Determine the fractional part 
  fractionaldivider = integerdivider - (100 * (tmpreg >> 4));
  
  // Implement the fractional part in the register 
  if ((UART5->CR1 & USART_CR1_OVER8) != 0)	// 8 oversampling
  {
    tmpreg |= (((fractionaldivider * 8) + 50) / 100) & (0x07);
  }
  else 						// 16 oversampling
  {
    tmpreg |= (((fractionaldivider * 16) + 50) / 100) & (0x0F);
  }
  
  // Write to USART BRR register
  UART5->BRR = (uint16_t)tmpreg;
  
}

void UART4_BRR_Configuration(uint32_t USART_BaudRate)
{ 
  uint32_t tmpreg = 0x00;
  uint32_t APB2clock = 42000000;	//PCLK2_Frequency
  uint32_t integerdivider = 0x00;
  uint32_t fractionaldivider = 0x00;
  
  // Determine the integer part 
  if ((UART4->CR1 & USART_CR1_OVER8) != 0) // USART_CR1_OVER8=(1<<15)
  {                                         // USART1->CR1.OVER8 = 1 (8 oversampling)
    // Computing 'Integer part' when the oversampling mode is 8 Samples 
    integerdivider = ((25 * APB2clock) / (2 * USART_BaudRate));    
  }
  else  // USART1->CR1.OVER8 = 0 (16 oversampling)
  {	// Computing 'Integer part' when the oversampling mode is 16 Samples 
    integerdivider = ((25 * APB2clock) / (4 * USART_BaudRate));    
  }
  tmpreg = (integerdivider / 100) << 4;
  
  // Determine the fractional part 
  fractionaldivider = integerdivider - (100 * (tmpreg >> 4));
  
  // Implement the fractional part in the register 
  if ((UART4->CR1 & USART_CR1_OVER8) != 0)	// 8 oversampling
  {
    tmpreg |= (((fractionaldivider * 8) + 50) / 100) & (0x07);
  }
  else 						// 16 oversampling
  {
    tmpreg |= (((fractionaldivider * 16) + 50) / 100) & (0x0F);
  }
  // Write to USART BRR register
  UART4->BRR = (uint16_t)tmpreg;
}

void _EXTI_Init(void)
{
  
  RCC->AHB1ENR 	|= (1<<3);	// RCC_AHB1ENR GPIOH Enable
  RCC->APB2ENR 	|=(1<<14);	// Enable System Configuration Controller Clock
  
  
  GPIOD->MODER     &= ~0x00F00000;      // GPIOD PIN11~PIN10 Input mode (reset state)   
  
  SYSCFG->EXTICR[2] |= 0x3300; 	// EXTI8~11�� ���� �ҽ� �Է��� GPIOH�� ����
  SYSCFG->EXTICR[3] |= 0x0000; 	// EXTI12~15�� ���� �ҽ� �Է��� GPIOH�� ����				
  
  EXTI->FTSR |= (1<<11);		// EXTI8~15Falling Trigger Enable (SW0~7)
  EXTI->FTSR |= (1<<10);		// EXTI8~15Falling Trigger Enable (SW0~7)
  
  EXTI->IMR  |= (1<<11);  	//  EXTI8~15 ���ͷ�Ʈ mask (Interrupt Enable) ����
  EXTI->IMR  |= (1<<10);  	
  // NVIC->ISER[0] |= ( 1 << 23 );   // INT NO.23
  NVIC->ISER[1] |= ( 1 << 40-32 );   // INT NO.40
  
}
void EXTI15_10_IRQHandler(void)		
{
  
  
  if(EXTI->PR & 0x0800)           // EXTI11 Interrupt Pending(�߻�) ����?
  {
    
    EXTI->PR |= 0x0800; 	  // Pending bit Clear (clear�� ���ϸ� ���ͷ�Ʈ ������ �ٽ� ���ͷ�Ʈ �߻�)
    
    feedback_flag =1 ;
    // �׻� ���� �ϰ� 2�� ���� �۵��ϱ� 
    
  }
  
  else if(EXTI->PR & 0x0400)           // EXTI10 Interrupt Pending(�߻�) ����?
  {
    
    EXTI->PR |= 0x0400; 	  // Pending bit Clear (clear�� ���ϸ� ���ͷ�Ʈ ������ �ٽ� ���ͷ�Ʈ �߻�)
    
    
    feedback_flag =2 ;
    // �׻� ���� �ϰ� 2�� ���� �۵��ϱ� 
    
  }
  
  
}

void USART2_BRR_Configuration(uint32_t USART_BaudRate)
{ 
  uint32_t tmpreg = 0x00;
  uint32_t APB1clock = 42000000;	//PCLK1_Frequency
  uint32_t integerdivider = 0x00;
  uint32_t fractionaldivider = 0x00;
  //--------------------------------------UART4�� -----------
  // Determine the integer part 
  // Determine the integer part 
  if ((USART2->CR1 & USART_CR1_OVER8) != 0) // USART_CR1_OVER8=(1<<15)
  {                                         // UART5->CR1.OVER8 = 1 (8 oversampling)
    // Computing 'Integer part' when the oversampling mode is 8 Samples 
    integerdivider = ((25 * APB1clock) / (2 * USART_BaudRate));    
  }
  else  // UART5->CR1.OVER8 = 0 (16 oversampling)
  {	// Computing 'Integer part' when the oversampling mode is 16 Samples 
    integerdivider = ((25 * APB1clock) / (4 * USART_BaudRate));    
  }
  tmpreg = (integerdivider / 100) << 4;
  
  // Determine the fractional part 
  fractionaldivider = integerdivider - (100 * (tmpreg >> 4));
  
  // Implement the fractional part in the register 
  if ((USART2->CR1 & USART_CR1_OVER8) != 0)	// 8 oversampling
  {
    tmpreg |= (((fractionaldivider * 8) + 50) / 100) & (0x07);
  }
  else 						// 16 oversampling
  {
    tmpreg |= (((fractionaldivider * 16) + 50) / 100) & (0x0F);
  }
  
  // Write to USART BRR register
  USART2->BRR = (uint16_t)tmpreg;                                            
}


void TIMER3_PWM_Init(void) // ���� ������
{   
  // TIM3 CH1 : PC6 
  // Clock Enable : GPIOC & TIMER3
  RCC->AHB1ENR   |= (1<<2);   // GPIOC CLOCK Enable
  RCC->APB1ENR    |= (1<<1);   // TIMER3 CLOCK Enable 
  
  // PC6�� ��¼����ϰ� Alternate function(TIM3_CH1)���� ��� ���� : PWM ���
  GPIOC->MODER    |= (2<<2*6);   // 0Output Alternate function mode               
  GPIOC->OSPEEDR    |= (3<<2*6);   //  Output speed (100MHz High speed)
  GPIOC->OTYPER   &= ~(1<<6);   // PC6 Output type push-pull (reset state)
  GPIOC->PUPDR   |= (1<<2*6);   // PC6 Pull-up
  GPIOC->AFR[0]   |= (2<<4*6);   // (AFR[1].(3~0)=0b0010): Connect TIM4 pins(PC6) to AF2(TIM3..5)
  
  
  //// PC7�� ��¼����ϰ� Alternate function(TIM3_CH2)���� ��� ���� : PWM ���
  GPIOC->MODER    |= (2<<2*7);   // 0Output Alternate function mode               
  GPIOC->OSPEEDR    |= (3<<2*7);   //  Output speed (100MHz High speed)
  GPIOC->OTYPER   &= ~(1<<7);   // PC7 Output type push-pull (reset state)
  GPIOC->PUPDR   |= (1<<2*7);   //PB7 Pull-up
  GPIOC->AFR[0]   |= (2<<4*7);   // 0x00000002 (AFR[1].(3~0)=0b0010): Connect TIM4 pins(PB8) to AF2(TIM3..5)
  
  
  
  // TIM4 Channel 3 : PWM 1 mode
  // Assign 'PWM Pulse Period'
  TIM3->PSC   = 42-1;   // Prescaler 84,000,000Hz/42 = 1,000,000 Hz(0.1ms)  (1~65536)
  TIM3->ARR   = 2000-1;   // Auto reload  (0.1ms * 100 = 10ms : PWM Period)
  
  // Setting CR1 : 0x0000 (Up counting)
  TIM3->CR1 &= ~(1<<4);   // DIR=0(Up counter)(reset state)
  TIM3->CR1 &= ~(1<<1);   // UDIS=0(Update event Enabled): By one of following events
  //   - Counter Overflow/Underflow, 
  //    - Setting the UG bit Set,
  //   - Update Generation through the slave mode controller 
  TIM3->CR1 &= ~(1<<2);   // URS=0(Update event source Selection): one of following events
  //   - Counter Overflow/Underflow, 
  //    - Setting the UG bit Set,
  //   - Update Generation through the slave mode controller 
  TIM3->CR1 &= ~(1<<3);   // OPM=0(The counter is NOT stopped at update event) (reset state)
  TIM3->CR1 &= ~(1<<7);   // ARPE=0(ARR is NOT buffered) (reset state)
  TIM3->CR1 &= ~(3<<8);    // CKD(Clock division)=00(reset state)
  TIM3->CR1 &= ~(3<<5);    // CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)
  // Center-aligned mode: The counter counts Up and DOWN alternatively
  
  // Define the corresponding pin by 'Output'  
  // CCER(Capture/Compare Enable Register) : Enable "Channel 3" 
  TIM3->CCER   |= (1<<0);   // CC3E=1: OC3(TIM4_CH3) Active(Capture/Compare 3 output enable)
  TIM3->CCER   &= ~(1<<1);   // CC3P=0: CC3 Output Polarity (OCPolarity_High : OC3���� �������� ���)
  
  TIM3->CCER   |= (1<<4);   // CC3E=1: OC3(TIM4_CH3) Active(Capture/Compare 3 output enable)
  TIM3->CCER   &= ~(1<<5);   // CC3P=0: CC3 Output Polarity (OCPolarity_High : OC3���� �������� ���)
  
  // Duty Ratio 
  TIM3->CCR1   =  PWM_1;      // CCR1 value
  TIM3->CCR2   =  PWM_2;      // CCR2 value
  
  map(PWM_1,0, 2000,0,100);
  // 'Mode' Selection : Output mode, PWM 1
  // CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2
  TIM3->CCMR1    &= ~(3<<0);    // CC1S(CC1 channel)= '0b00' : Output 
  TIM3->CCMR1    |= (1<<3);    // OC1PE=1: Output Compare 1 preload Enable
  TIM3->CCMR1   |= (7<<4);   // OC1M=0b110: Output compare 1 mode: PWM 1 mode
  TIM3->CCMR1   |= (1<<7);   // OC1CE=1: Output compare 1 Clear enable
  
  TIM3->CCMR1    &= ~(3<<8);    // CC2S(CC2 channel)= '0b00' : Output 
  TIM3->CCMR1    |= (1<<11);    // OC2PE=1: Output Compare 2 preload Enable
  TIM3->CCMR1   |= (7<<12);   // OC2M=0b110: Output compare 2 mode: PWM 1 mode
  TIM3->CCMR1   |= (1<<15);   // OC2CE=1: Output compare 2 Clear enable
  
  //Counter TIM3 enable
  TIM3->CR1   |= (1<<7);   // ARPE: Auto-reload preload enable 
  TIM3->CR1   |= (1<<0);   // CEN: Counter TIM4 enable
}

/*
void TIMER4_PWM_Init(void)  // ���Ͼ� ���߿����Ϳ�                       -> ���Ͼ� -> ���ڴ� �� ��� ���� 
{   
// TIM4 CH1 : PB6
// Clock Enable : GPIOB & TIMER4
RCC->AHB1ENR	|= (1<<1);	// GPIOB CLOCK Enable
RCC->APB1ENR 	|= (1<<2);	// TIMER4 CLOCK Enable 

// PB7�� ��¼����ϰ� Alternate function(TIM4_CH1)���� ��� ���� : PWM ���
GPIOB->MODER 	|= (2<<2*7);	// 0x00020000 PB8 Output Alternate function mode					
GPIOB->OSPEEDR 	|= (3<<2*7);	// 0x00030000 PB8 Output speed (100MHz High speed)
GPIOB->OTYPER	&= ~(1<<7);	// PB8 Output type push-pull (reset state)
GPIOB->PUPDR	|= (1<<2*7);	// 0x00010000 PB8 Pull-up
GPIOB->AFR[0]	|= (2<<4*7);	// 0x00000002 (AFR[1].(3~0)=0b0010): Connect TIM4 pins(PB8) to AF2(TIM3..5)

// PB8�� ��¼����ϰ� Alternate function(TIM4_CH2)���� ��� ���� : PWM ���
GPIOB->MODER 	|= (2<<2*8);	// 0x00020000 PB8 Output Alternate function mode					
GPIOB->OSPEEDR 	|= (3<<2*8);	// 0x00030000 PB8 Output speed (100MHz High speed)
GPIOB->OTYPER	&= ~(1<<8);	// PB8 Output type push-pull (reset state)
GPIOB->PUPDR	|= (1<<2*8);	// 0x00010000 PB8 Pull-up
GPIOB->AFR[0]	|= (2<<4*8);	// 0x00000002 (AFR[1].(3~0)=0b0010): Connect TIM4 pins(PB7) to AF2(TIM3..5)

// TIM4 Channel 1 : PWM 1 mode
// Assign 'PWM Pulse Period'
TIM4->PSC	= 4200-1;	// Prescaler 84,000,000Hz/4200 = 10,000 Hz(0.1ms)  (1~65536)
TIM4->ARR	= 100-1;	// Auto reload  (0.1ms * 100 = 10ms : PWM Period)

// Setting CR1 : 0x0000 (Up counting)
TIM4->CR1 &= ~(1<<4);	// DIR=0(Up counter)(reset state)
TIM4->CR1 &= ~(1<<1);	// UDIS=0(Update event Enabled): By one of following events
//	- Counter Overflow/Underflow, 
// 	- Setting the UG bit Set,
//	- Update Generation through the slave mode controller 
TIM4->CR1 &= ~(1<<2);	// URS=0(Update event source Selection): one of following events
//	- Counter Overflow/Underflow, 
// 	- Setting the UG bit Set,
//	- Update Generation through the slave mode controller 
TIM4->CR1 &= ~(1<<3);	// OPM=0(The counter is NOT stopped at update event) (reset state)
TIM4->CR1 &= ~(1<<7);	// ARPE=0(ARR is NOT buffered) (reset state)
TIM4->CR1 &= ~(3<<8); 	// CKD(Clock division)=00(reset state)
TIM4->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel)=00 (Edge-aligned mode) (reset state)

// Center-aligned mode: The counter counts Up and DOWN alternatively

// Define the corresponding pin by 'Output'  
// CCER(Capture/Compare Enable Register) : Enable "Channel 3" 
TIM4->CCER	|= (1<<0);	// CC1E=1: OC1(TIM4_CH1) Active(Capture/Compare 1 output enable)
TIM4->CCER	&= ~(1<<1);	// CC1P=0: CC1 Output Polarity (OCPolarity_High : OC1���� �������� ���)
TIM4->CCER	|= (1<<4);	// CC2E=1: OC2(TIM4_CH2) Active(Capture/Compare 2 output enable)
TIM4->CCER	&= ~(1<<5);	// CC2P=0: CC2 Output Polarity (OCPolarity_High : OC2���� �������� ���)

// Duty Ratio 
TIM4->CCR1	= 95;		// CCR1 value
TIM4->CCR2	= 95;		// CCR2 value

// 'Mode' Selection : Output mode, PWM 1
// CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2
TIM4->CCMR1 	&= ~(3<<0); 	// CC1S(CC1 channel)= '0b00' : Output 
TIM4->CCMR1 	|= (1<<3); 	// OC1PE=1: Output Compare 1 preload Enable
TIM4->CCMR1	|= (6<<4);	// OC1M=0b110: Output compare 1 mode: PWM 1 mode
TIM4->CCMR1	|= (1<<7);	// OC1CE=1: Output compare 1 Clear enable

TIM4->CCMR1 	&= ~(3<<8); 	// CC2S(CC2 channel)= '0b00' : Output 
TIM4->CCMR1 	|= (1<<11); 	// OC2PE=1: Output Compare 2 preload Enable
TIM4->CCMR1	|= (6<<12);	// OC2M=0b110: Output compare 2 mode: PWM 1 mode
TIM4->CCMR1	|= (1<<15);	// OC2CE=1: Output compare 2 Clear enable

//Counter TIM4 enable
TIM4->CR1	|= (1<<7);	// ARPE: Auto-reload preload enable
TIM4->CR1	|= (1<<0);	// CEN: Counter TIM4 enable
}

*/
void TIMER1_PWM_Init(void)
{  
  // �����޽�(PWM)��:PA0(TIM1 CH1), ���͹���(DIR)��:PE9
  // Clock Enable : GPIOE & TIMER1
  RCC->AHB1ENR	|= (1<<4);	// GPIOE Enable
  RCC->APB2ENR 	|= (1<<0);	// TIMER1 Enable 
  
  // PE9�� ��¼����ϰ� Alternate function(TIM1_CH1)���� ��� ���� : PWM ���
  GPIOE->MODER 	|= (2<<2*9);	// PE0 Output Alternate function mode					
  GPIOE->OSPEEDR 	|= (3<<2*9);	// PE0 Output speed (100MHz High speed)
  GPIOE->OTYPER	&= ~(1<<9);	// PA0 Output type push-pull (reset state)
  GPIOE->AFR[1]	|= (1<<4); 	// 0x00000002	(AFR[0].(3~0)=0b0010): Connect TIM5 pins(PA0) to AF2(TIM3..5)
  
  // PE10,PE11�� GPIO  ��¼��� : Dir (���͹���)
  GPIOE->MODER 	|= (1<<2*10);	// PE10 Output  mode					
  GPIOE->OSPEEDR 	|= (1<<2*10);	// PE10 Output speed (25MHz High speed)
  GPIOE->OTYPER	&= ~(1<<10);	// PE10 Output type push-pull (reset state)
  
  GPIOE->MODER 	|= (1<<2*11);	// PE10 Output  mode					
  GPIOE->OSPEEDR 	|= (1<<2*11);	// PE10 Output speed (25MHz High speed)
  GPIOE->OTYPER	&= ~(1<<11);	// PE10 Output type push-pull (reset state)
  
  // TIM1 Channel 1 : PWM 1 mode
  // Assign 'PWM Pulse Period'
  TIM1->PSC	= 84-1;	// Prescaler 168,000,000Hz/84 = 1,000,000 Hz(0.1ms)  (1~65536)
  TIM1->ARR	= 2000-1;	// Auto reload  (0.1ms * 100 = 10ms : PWM Period) ()
  
  // Define the corresponding pin by 'Output'  ry
  // CCER(Capture/Compare Enable Register) : Enable "Channel 1" 
  TIM1->CCER	|= (1<<0);	// CC1E=1: OC1(TIM5_CH1) Active(Capture/Compare 1 output enable)
  // �ش���(40��)�� ���� ��ȣ���
  TIM1->CCER	&= ~(1<<1);	// CC1P=0: CC1 output Polarity High (OC1���� �������� ���)
  
  // Duty Ratio 
  TIM1->CCR1	= 200;		// CCR1 value
  
  // 'Mode' Selection : Output mode, PWM 1
  // CCMR1(Capture/Compare Mode Register 1) : Setting the MODE of Ch1 or Ch2
  TIM1->CCMR1 	&= ~(3<<0); 	// CC1S(CC1 channel)='0b00' : Output 
  TIM1->CCMR1 	|= (1<<3); 	// OC1PE=1: Output Compare 1 preload Enable
  
  TIM1->CCMR1	|= (6<<4);	// OC1M: Output compare 1 mode: PWM 1 mode
  TIM1->CCMR1	|= (1<<7);	// OC1CE: Output compare 1 Clear enable
  
  // CR1 : Up counting & Counter TIM1 enable
  TIM1->CR1 	&= ~(1<<4);	// DIR: Countermode = Upcounter (reset state)
  TIM1->CR1 	&= ~(3<<8);	// CKD: Clock division = 1 (reset state)
  TIM1->CR1 	&= ~(3<<5); 	// CMS(Center-aligned mode Sel): No(reset state)
  TIM1->CR1	|= (1<<7);	// ARPE: Auto-reload preload enable
  TIM1->BDTR      |= (1<<15);
  TIM1->CR1	|= (1<<0);	// CEN: Counter TIM1 enable
}
void TIMER7_Init(void)                          // APB1 -> 42mB 
{
  NVIC_SetPriority(TIM7_IRQn ,16 );
  
  RCC->APB1ENR |= (1<<5);	//RCC_APB1ENR TIMER3 Enable   // 84M Hz 
  
  NVIC->ISER[1] |= ( 1 << 55-32 ); // Enable Timer3 global Interrupt
  
  TIM7->PSC = 420 -1;	// Prescaler 84,000,000Hz/420 = 200000 Hz (0.005ms)  (1~65536)
  TIM7->ARR = 20000-1;	        // Auto reload  0.005ms *20000 = 100ms
  
  TIM7->CR1 &= ~(1<<4);	// Upcounter(reset state)  // 4�� ��Ʈ�� 0���� �ϴ°�  �� ī���� ���ڴ�. 
  TIM7->CR1 &= ~(3<<8); 	// CKD(Clock division)=1(reset state)                   // 11 �� 8~9 ���� ���� // ~ �̴� 0�� ���� 
  TIM7->CR1 &= ~(3<<5); 	// CMS(Center-aligned mode Sel): No(reset state)
  
  TIM7->EGR |=(1<<0);	// Update generation    
  
  TIM7->DIER |= (1<<0);	// Enable the Tim3 Update interrupt  // ���ͷ�Ʈ ��� �ϰڴ�. 
  TIM7->CR1 |= (1<<0);	// Enable the Tim3 Counter     // Ŭ�� �߻� 
}
void TIM7_IRQHandler(void)  	// 1ms Interrupt
{
  
  TIM7->SR &= ~(1<<0);	// Interrupt flag Clear                 // ����  �ʱ�ȭ  1�̸� ���ͷ�Ʈ �߻� �ߴٴ°��� ���� �ֱ� ������ 0 ���� �ʱ�ȭ ���� 
  
  /*********************** ��Ʈ ��� �����̰� �ϴ� �κ� **********************/
  //          if(senser_play == 1 && senser_flag ==0 && senser_delay_flag == 0)
  //          {
  //                 senser_time++;
  //                 if(     senser_time == 30)
  //                 {
  //                   senser_flag = 1;
  //                   GPIOG->ODR |= (1<<5);                     //enable off
  //                   GPIOG->ODR |= (1<<2);                     //enable off
  //                   senser_delay_flag = 1;
  //                   senser_time = 0;
  //                 }
  //          }
  //          else if (senser_play == 1 && senser_flag ==2 && senser_delay_flag == 0)
  //          {
  //            senser_time++;
  //            if(senser_time == 20)
  //            {
  //              senser_flag = 3;
  //              
  //                   GPIOG->ODR |= (1<<5);                     //enable off
  //                   GPIOG->ODR |= (1<<2);                     //enab
  //              senser_delay_flag = 1;
  //              senser_time = 0;
  //            }
  //          }
  //          else if(senser_play == 1 && senser_flag ==4 && senser_delay_flag == 0)
  //          {
  //            senser_time++;
  //            if(senser_time == 30)
  //            {
  //              senser_flag = 5;
  //              
  //                   GPIOG->ODR |= (1<<5);                     //enable off
  //                   GPIOG->ODR |= (1<<2);                     //enab
  //              senser_delay_flag = 1;
  //              senser_time = 0;
  //            }
  //          }
  //              else if(senser_play == 1 && senser_flag ==6 && senser_delay_flag == 0)
  //          {
  //            senser_time++;
  //            if(senser_time == 20)
  //            {
  //              senser_flag = 7;
  //              
  //                   GPIOG->ODR |= (1<<5);                     //enable off
  //                   GPIOG->ODR |= (1<<2);                     //enab
  //              senser_delay_flag = 1;
  //              senser_time = 0;
  //            }
  //          }
  //          
  //          
  //          if(senser_delay_flag == 1)
  //          {
  //            GPIOE->ODR |= (1<<2);               // off 
  //            senser_delay++;
  //            if(senser_delay == 10)
  //            {
  //              senser_delay = 0;
  //              
  //                   GPIOG->ODR |= (1<<5);                     //enable off
  //                   GPIOG->ODR |= (1<<2);                     //enab
  //            senser_delay_flag = 0;
  //            }
  //            
  //          }
  //          
  //       /*********************** ��Ʈ ��� �����̰� �ϴ� �κ� **********************/     
  
  
  // ���� ������ û�� ���� ���� �ϴ� �� 
  
  
  // ���ڱ� ���� ���� �� �ʱ�ȭ 
  /*
  if(Auto_flag  == 1)
  {
  S_flag++;
  if(S_flag == 100)
  {
  
  err = pid_Kp_term = pid_Ki_term = pid_Kd_term = pid_control = d_err = I_err_dt = prev_err   = 0.000;
  goal_PWM  = 0 ;
  S_flag = 0;
}
}
  */
  
  
  // GPS �� �����ϴ� �ڵ� 
  if(GPS_Point == 1)
  {
    GPS_Stop++;
    
    if(GPS_Stop >= 50)
    {
      if(save_p >= 4)
      {
        save_p = 0;
        
      }
      else
      {
        
        // save_latitude[save_p] = sum ;               // ����
        //save_longtitude[save_p++] = sum2;              // �浵 
        save_p++;                     // �׽�Ʈ �� ���� ��� �� �� 
        if(save_p >= 2 )
        {
          // c_radians_latitude=sum *(3.141592/180);                            // ���� ���� ����
          //c_radians_longtitude = sum2 *(3.141592/180);                         // ���� ���� �浵 
          if(save_p <4)                 // ���� ���� ���� 
          {
            radians_latitude = save_latitude[point]*(3.141592/180);           // ���� ���� 0
            radians_longtitude = save_longtitude[point++]*(3.141592/180);               // ���� �浵 
            g_radians_latitude = save_latitude[point]*(3.141592/180)   ;               // ��ǥ ���� ����
            g_radians_longtitude = save_longtitude[point]*(3.141592/180);             // ��ǥ ���� �浵 
          }
          else if (save_p >= 4)                 // ó���� ��ǥ���������� ���� ���� .
          {
            radians_latitude = save_latitude[0]*(3.141592/180);           // ���� ���� 0
            radians_longtitude = save_longtitude[0]*(3.141592/180);               // ���� �浵 
            g_radians_latitude = save_latitude[++point]*(3.141592/180)   ;               // ��ǥ ���� ����
            g_radians_longtitude = save_longtitude[point]*(3.141592/180);             // ��ǥ ���� �浵 
          }
          
          
          if(point == 1)
          {
            radians_space = acos(sin(radians_latitude)*sin(g_radians_latitude) + cos(radians_latitude)*cos(g_radians_latitude)*cos(radians_longtitude-g_radians_longtitude));
            // ��� - ��ǥ ���� �Ÿ��� ���� //
            real_space_H = radians_space*6366692.0724;                     // ���� �Ÿ�
          }
          else if(point == 2)
          {
            radians_space = acos(sin(radians_latitude)*sin(g_radians_latitude) + cos(radians_latitude)*cos(g_radians_latitude)*cos(radians_longtitude-g_radians_longtitude));
            // ��� - ��ǥ ���� �Ÿ��� ���� //
            real_space_V = radians_space*6366692.0724;                     // ���� �Ÿ� 
          }
          else if (point == 3)
          {
            radians_space = acos(sin(radians_latitude)*sin(g_radians_latitude) + cos(radians_latitude)*cos(g_radians_latitude)*cos(radians_longtitude-g_radians_longtitude));
            // ��� - ��ǥ ���� �Ÿ��� ���� //
            real_space_Turm = radians_space*6366692.0724;                     // ��ǥ����  �Ÿ� 
            
          }
        }
        
        
        
      }
      GPS_Stop  = 0;
      GPS_Point = 0;
    }
    
    
  }
  
  if(real_check_flag == 1)
  {
    system_time++;
    
    if(system_time >= 50)
    {
      check_p++;
      real_check_flag =0;
      system_time = 0;
    }
  }
  
  // RPM TIMER �׽�Ʈ 
  
  if(RPM_timer_flag == 1)                 //0.1s 
  {
    
    if(feedback_flag == 1 ||  feedback_flag == 2 )
    {
      save_RPM_m = RPM_m ; 
      
    }
    else{
      rpm = map_int(PWM_2,900,1500,4250,4450);
      
      RPM = rpm*0.0219181/1000 ;  // 4200 ~ 4500  ���� ��               //cm �Դϴ�.       = 0.095343735 m/0.1s
      
      if(feedback_flag == 1 ||  feedback_flag == 2)
      {
        save_RPM_m = RPM_m ; 
        
      }
      RPM_m += RPM; 
      
      sprintf(str_RPM, "%.8f", RPM_m);
      
      
      //                                if(RPM_m >=  real_space_H+0.5)              // real_space == ��ǥ���� �޾� ���°Ÿ� �� 
      //                                {
      //                                                RPM_timer_test_flag ++;
      
      //                                  GPIOG->ODR |= (1<<6);                   //brk on
      //                                  GPIOG->ODR |= (1<<3);
      //                                  GPIOE->ODR |= (1<<2);            // �귯�� enable off 
      //                                   RPM_timer_flag  = 0;
      //                                   RPM_m = 0;
      //                                senser_delay_flag = 1;
      //                  
      //                                }
      
      
      if(RPM_m >=  test_m+0.5)              // real_space == ��ǥ���� �޾� ���°Ÿ� �� 
      {
        RPM_timer_test_flag ++;
        
        GPIOG->ODR |= (1<<6);                   //brk on
        GPIOG->ODR |= (1<<3);
        GPIOE->ODR |= (1<<2);            // �귯�� enable off 
        RPM_timer_flag  = 0;
        RPM_m = 0;
        feedback_flag = 0;
        senser_delay_flag = 1;
      }
    }
  }
  
  if(senser_delay_flag == 1)                      // ���� ��ǥ���� ������ 2�� ����
  {
    
    GPIOE->ODR |= (1<<2);              // off 
    senser_delay++;
    if(senser_delay == 20)
    {
      senser_delay = 0;
      GPIOG->ODR |= (1<<6);                   //brk on
      GPIOG->ODR |= (1<<3);
      senser_delay_flag = 0;
    }
  }
  else if(senser_delay_flag == 2)                        // ���� �̵� �� ���� ���ð� 1.5�� 
  {
    
    GPIOE->ODR |= (1<<2);               //brush off 
    GPIOG->ODR &= ~(1<<6);                   //brk on
    GPIOG->ODR &= ~(1<<3);
    // 600 ���� 
    rpm = 2000;
    RPM = rpm*0.0219181/1000 ;  // 4200 ~ 4500  ���� ��               //cm �Դϴ�.       = 0.095343735 m/0.1s
    RPM_m += RPM; 
    sprintf(str_RPM, "%.8f", RPM_m);
    
    
    //          if(RPM_m >=  real_space_V+0.5)
    //          {
    //
    // 
    //                GPIOG->ODR |= (1<<6);                   //brk off
    //                GPIOG->ODR |= (1<<3);
    //                                 RPM_m = 0;
    //            senser_delay_flag = 0;
    //          }
    
    
    
    if(RPM_m >=  test_mv +0.5)
    {
      
      
      GPIOG->ODR |= (1<<6);                   //brk off
      GPIOG->ODR |= (1<<3);
      RPM_m = 0;
      feedback_flag = 0;
      senser_delay_flag = 0;
    }
    
  }
  else if(senser_delay_flag == 3)                         // 2�ʰ� ���� 
  {
    GPIOE->ODR &= ~(1<<2);              // �귯�� on
    GPIOG->ODR |= (1<<4);                  // ������ CW
    GPIOG->ODR &= ~(1<<1);                // ���� CCW
    GPIOG->ODR &= ~(1<<6);                   //brk on
    GPIOG->ODR &= ~(1<<3);                              // brk on
    senser_delay++;
    if(senser_delay == 30)
    {
      senser_delay = 0;
      GPIOE->ODR |= (1<<2);              // �귯�� off
      GPIOG->ODR |= (1<<6);                   //brk off
      GPIOG->ODR |= (1<<3);
      senser_delay_flag = 0;
    }
  }
  else if(senser_delay_flag == 4)         // 2�ʰ� ���� 
  {
    
    GPIOG->ODR &= ~(1<<4);                  // ������ CCW
    GPIOG->ODR |= (1<<1);                    // ���� CW
    
    GPIOG->ODR &= ~(1<<6);                   //brk on
    GPIOG->ODR &= ~(1<<3);                              // brk on
    senser_delay++;
    if(senser_delay == 30)
    {
      senser_delay = 0;
      GPIOE->ODR |= (1<<2);              // �귯�� off
      GPIOG->ODR |= (1<<6);                   //brk off
      GPIOG->ODR |= (1<<3);
      senser_delay_flag = 0;
    }
    
  }
  else if(senser_delay_flag == 5)         // 4�ʰ� ���� 
  {
    
    GPIOG->ODR &= ~(1<<4);                  // ������ CCW
    GPIOG->ODR |= (1<<1);                    // ���� CW
    
    GPIOG->ODR &= ~(1<<6);                   //brk on;;;
    GPIOG->ODR &= ~(1<<3);                              // brk on
    senser_delay++;
    if(senser_delay == 40)
    {
      senser_delay = 0;
      GPIOE->ODR |= (1<<2);              // �귯�� off
      GPIOG->ODR |= (1<<6);                   //brk off
      GPIOG->ODR |= (1<<3);
      senser_delay_flag = 0;
    }
    
  }
  else if(senser_delay_flag == 6)         // 4�ʰ� ���� 
  {
    
    GPIOG->ODR  |= (1<<4);                  // ������ CW
    GPIOG->ODR  &= ~(1<<1);                    // ���� CCW
    
    GPIOG->ODR &= ~(1<<6);                   //brk on
    GPIOG->ODR &= ~(1<<3);                              // brk on
    senser_delay++;
    if(senser_delay == 40)
    {
      senser_delay = 0;
      GPIOE->ODR |= (1<<2);              // �귯�� off
      GPIOG->ODR |= (1<<6);                   //brk off
      GPIOG->ODR |= (1<<3);
      senser_delay_flag = 0;
    }
    
  }
}

void DelayMS(unsigned short wMS)
{	register unsigned short i;
for (i=0; i<wMS; i++)
DelayUS(1000);  // 1000us => 1ms
}
void DelayUS(unsigned short wUS)
{	volatile int Dly = (int)wUS*17;
for(; Dly; Dly--);
}

void Linear_UP(void)
{
  GPIOE->ODR &= ~ (15<<3);
  GPIOE->ODR |=  (1<<3);
  GPIOE->ODR |=  (1<<5);
}

void Linear_DOWN(void)
{
  
  GPIOE->ODR &= ~ (15<<3);
  GPIOE->ODR |= (1<<4);
  GPIOE->ODR |= (1<<6);
}


uint8_t key_flag = 0;
uint16_t KEY_Scan(void)	// input key SW0 - SW7 
{ 
  uint16_t key;
  key = GPIOH->IDR & 0xFF00;	// any key pressed ?
  if(key == 0xFF00)		// if no key, check key off
  {  	if(key_flag == 0)
    return key;
  else
  {	DelayMS(10);
  key_flag = 0;
  return key;
  }
  }
  else				// if key input, check continuous key
  {	if(key_flag != 0)	// if continuous key, treat as no key input
    return 0xFF00;
  else			// if new key,delay for debounce
  {	key_flag = 1;
  DelayMS(10);
  return key;
  }
  }
}
void LCD_cycle()
{
  //PWM ���� 
  //-----------------------------------------------------------------------------------------
  
  P_PWM_1 = map(PWM_1,0,2000,0,100);
  P_PWM_2 = map(PWM_2,0,2000,0,100);
  //  
  TM_ILI9341_Putc(75, 70, (int)(PWM_1/1000)+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(85, 70, (int)(PWM_1%1000/100)+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(95, 70, (int)(PWM_1%100/10)+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(105, 70, (int)(PWM_1%10)+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  
  TM_ILI9341_Putc(190, 70, (int)(PWM_2/1000)+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(200, 70, (int)(PWM_2%1000/100)+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(210, 70, (int)(PWM_2%100/10)+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(220, 70, (int)(PWM_2%10)+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  
  TM_ILI9341_Putc(75, 90, P_PWM_1/100+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(85, 90, P_PWM_1%100/10+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(95, 90, P_PWM_1%10+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(105,90, '%', &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  
  TM_ILI9341_Putc(190, 90, P_PWM_2/100+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(200, 90, P_PWM_2%100/10+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(210, 90, P_PWM_2%10+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(220, 90, '%', &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  
  //���ڱ� ���� 
  //-----------------------------------------------------------------------------------------
  if(yaw_Z[5][0] == ' ')
  {
    Z_deg = -Z_data;
    TM_ILI9341_Putc(80, 40, '-', &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
    TM_ILI9341_Putc(90, 40, ((int)(Z_data*100)/10000)+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
    TM_ILI9341_Putc(100, 40, ((int)(Z_data*100)%10000/1000)+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
    TM_ILI9341_Putc(110, 40, ((int)(Z_data*100)%1000/100)+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
    TM_ILI9341_Putc(120, 40, '.', &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
    TM_ILI9341_Putc(130, 40, ((int)(Z_data*100)%100/10)+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
    TM_ILI9341_Putc(140, 40, ((int)(Z_data*100)%10)+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  }
  else
  {
    Z_deg = Z_data ;
    TM_ILI9341_Putc(80, 40, ' ', &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
    TM_ILI9341_Putc(90, 40, ((int)(Z_data*100)/10000)+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
    TM_ILI9341_Putc(100, 40, ((int)(Z_data*100)%10000/1000)+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
    TM_ILI9341_Putc(110, 40, ((int)(Z_data*100)%1000/100)+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
    TM_ILI9341_Putc(120, 40, '.', &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
    TM_ILI9341_Putc(130, 40, ((int)(Z_data*100)%100/10)+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
    TM_ILI9341_Putc(140, 40, ((int)(Z_data*100)%10)+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  }
  // PID ���� �� 
  //-----------------------------------------------------------------------------------------
  //Kp
  TM_ILI9341_Putc(50, 120, (int)(pid_Kp*1000)/1000+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(60, 120, '.', &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(70, 120, (int)(pid_Kp*1000)%1000/100+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(80, 120, (int)(pid_Kp*1000)%100/10+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(90, 120, (int)(pid_Kp*1000)%10+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  
  //Ki
  TM_ILI9341_Putc(165, 120, (int)(pid_Ki*1000)/1000+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(175, 120, '.', &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(185, 120, (int)(pid_Ki*1000)%1000/100+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(195, 120, (int)(pid_Ki*1000)%100/10+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(205, 120, (int)(pid_Ki*1000)%10+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  //Kd
  TM_ILI9341_Putc(50, 140, (int)(pid_Kd*1000)/1000+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(60, 140, '.', &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(70, 140, (int)(pid_Kd*1000)%1000/100+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(80, 140, (int)(pid_Kd*1000)%100/10+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(90, 140, (int)(pid_Kd*1000)%10+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  //Delay
  TM_ILI9341_Putc(195, 140, delay/1000+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(205, 140, delay%1000/100+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(215, 140, delay%100/10+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(225, 140, delay%10+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  //Scale
  TM_ILI9341_Putc(80, 160, Scale/10+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(90, 160, Scale%10+0x30, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  
  // ����
  //-----------------------------------------------------------------------------------------
  TM_ILI9341_Puts(75, 220, str , &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  
  //�浵
  //-----------------------------------------------------------------------------------------
  TM_ILI9341_Puts(75, 240, str2, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  
  
  //Timer
  //------------------------------------------------------------------------------------
  
  TM_ILI9341_Putc(80, 190, S_flag/10+0x30 , &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(90, 190, S_flag%10+0x30 , &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  
  
  
  TM_ILI9341_Putc(110, 190, GPS_Stop/10+0x30 , &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(120, 190, GPS_Stop%10+0x30 , &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  
  TM_ILI9341_Putc(140, 190, save_p+0x30 , &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  
  TM_ILI9341_Putc(160, 190, point+0x30 , &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  
  //RPM
  //---------------------------------------------------------------------------------------
  
  TM_ILI9341_Puts(70, 280,str_RPM,&TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);         // ���� �ð� 
  
  TM_ILI9341_Puts(70, 260,str3,&TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);               // ���� �Ÿ� 
  
  
  //      TM_ILI9341_Putc(70, 220,(int)rpm/1000+0x30,&TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);               // ���� �Ÿ� 
  //     TM_ILI9341_Putc(80, 220,(int)rpm%1000/100+0x30,&TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);               // ���� �Ÿ� 
  //     TM_ILI9341_Putc(90, 220,(int)rpm%100/10+0x30,&TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);               // ���� �Ÿ� 
  //     TM_ILI9341_Putc(100, 220,(int)rpm%10+0x30,&TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);               // ���� �Ÿ� 
  
  
  //�׽�Ʈ 
  //-----------------------------------------------------------------------------------------
  
  //     TM_ILI9341_Puts(80, 210,str5,&TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);               // ���� �Ÿ�                     // ���� ���� 
  //     TM_ILI9341_Puts(80, 230,str6,&TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);               // ���� �Ÿ�                     // ����Ŭ Ƚ�� 
  
  //  TM_ILI9341_Putc(200, 10, 0x31, &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  TM_ILI9341_Putc(200, 230,senser_delay_flag+0x30,&TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);               // ���� �Ÿ� 
  TM_ILI9341_Puts(150, 160, str6, &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  
  
  
  
  
  
  
  //TM_ILI9341_Putc(80, 280, test_m/10+0x30 ,&TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  //TM_ILI9341_Putc(90, 280, test_m%10+0x30 ,&TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  //        TM_ILI9341_Putc(210, 260, c_flag/10+0x30 ,&TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  //       TM_ILI9341_Putc(220, 260, c_flag%10+0x30 ,&TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  //        TM_ILI9341_Putc(210, 280, senser_time/10+0x30 ,&TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  //        TM_ILI9341_Putc(220, 280, senser_time%10+0x30 ,&TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  //
  
  //-----------------------------------------------------------------------------------------
  
  
  //
  //-----------------------------------------------------------------------------------------
  
  
}
void LCD_lnit()
{
  SystemInit();
  // y �� �������� ���� ũ�� ���̴� 20 MAX : 300 pixel
  // x �� �������� ���� 1�� ���� 10;    MAX : 220 Pixel 
  //
  
  TM_ILI9341_Init();
  // ȸ�� 
  TM_ILI9341_Rotate(TM_ILI9341_Orientation_Portrait_1);
  
  // TM_Font_7x10             
  // TM_Font_11x18                    ��¥ũ��
  // TM_Font_16x26
  
  
  TM_ILI9341_Puts(10, 10, "FOD Cleaner", &TM_Font_16x26, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  
  TM_ILI9341_Puts(10, 40, "Z_deg:", &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  
  
  TM_ILI9341_Puts(10, 70, "PWM1:", &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  TM_ILI9341_Puts(125, 70, "PWM2:", &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  
  TM_ILI9341_Puts(10, 120, "Kp:", &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  TM_ILI9341_Puts(125, 120, "Ki:", &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  TM_ILI9341_Puts(10, 140, "Kd:", &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  TM_ILI9341_Puts(125, 140, "Delay:", &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  TM_ILI9341_Puts(10, 160, "Scale:", &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  
  
  TM_ILI9341_Puts(10, 190, "Timer:", &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  
  TM_ILI9341_Puts(10, 210, "Lat :", &TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  
  TM_ILI9341_Puts(10, 230, "Long:", &TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  
  TM_ILI9341_Puts(10, 260, "G_S:" ,&TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);               // ���� ����
  
  TM_ILI9341_Puts(10, 280, "C_S:" ,&TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);               // ���� ���� 
  //TM_ILI9341_Puts(10, 260, "RPM_r:" ,&TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  
  //TM_ILI9341_Puts(10, 280, "RPM_R:" ,&TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  
  //TM_ILI9341_Putc(150, 280, '1',&TM_Font_11x18, ILI9341_COLOR_BLACK, ILI9341_COLOR_YELLOW);
  
  //    
  //    TM_ILI9341_Puts(10, 280, "C_S:" ,&TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  //    TM_ILI9341_Puts(10, 300, "C_S:" ,&TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  //    
  //    TM_ILI9341_Puts(220, 280, ":" ,&TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
  //    TM_ILI9341_Putc(210, 300, 0x30 ,&TM_Font_11x18, ILI9341_COLOR_BLACK,ILI9341_COLOR_YELLOW);
}

