/*
project_Flying_WingF3_V1  
by: tinnakon kheowree  
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com
https://www.facebook.com/tinnakonza
*/
//motor Arduino boards 328P works on pins 3, 10, 11, 9
//uint8_t MOTOR_FrontL_PIN = 3;//PD3
uint8_t MOTOR_FrontL_PIN = 10;//PB2
//uint8_t MOTOR_BackL_PIN = 11;//PB3
uint8_t MOTOR_FrontR_PIN = 9;//PB1
    
#define PWM_FREQUENCY 300   //400 in Hz and 300 Hz
#define PWM_PRESCALER 8
#define PWM_COUNTER_PERIOD (F_CPU/PWM_PRESCALER/PWM_FREQUENCY)

float motor_FrontL = MINCOMMAND;
float motor_FrontR = MINCOMMAND;
//float motor_BackL = MINCOMMAND;
//float motor_BackR = MINCOMMAND;

void motor_command_all() 
{
  for (int j = 0 ; j <= 50 ; j++)
  {
   motor_FrontL = 1500;
   motor_FrontR = 1500;
   //motor_BackL = MINCOMMAND;
   //motor_BackR = MINCOMMAND;
   
   //OCR2B = motor_FrontL / 16 ;//PD3  1000-2000 to 128-256
   OCR1A = motor_FrontR * 2 ;//PB1
   OCR1B = motor_FrontL * 2 ;//PB2
   //OCR2A = motor_BackL / 16 ;//PB3
   delay(20);
  }
}
void motor_initialize() 
{
    DDRB = DDRB | B00001110;   // Set ports to output PB1-3
    DDRD = DDRD | B00001000;   // Set port to output PD3
        // Init PWM Timer 1  16 bit
    TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);
    TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
    ICR1 = PWM_COUNTER_PERIOD;
    // Init PWM Timer 2   8bit                                 // WGMn1 WGMn2 = Mode ? Fast PWM, TOP = 0xFF ,Update of OCRnx at BOTTOM
    //TCCR2A = (1<<WGM20)|(1<<WGM21)|(1<<COM2A1)|(1<<COM2B1);    // Clear OCnA/OCnB on compare match, set OCnA/OCnB at BOTTOM (non-inverting mode)
    //TCCR2B = (1<<CS22)|(1<<CS21);                              // Prescaler set to 256, that gives us a resolution of 16us
    // TOP is fixed at 255                                     // Output_PWM_Frequency = 244hz = 16000000/(256*(1+255)) = Clock_Speed / (Prescaler * (1 + TOP))
  motor_command_all();         // Initialise motors to MINCOMMAND (stopped)
}
void motor_command() 
{
   //OCR2B = motor_FrontLf / 16 ;//PD3  1000-2000 to 128-256
   OCR1A = motor_FrontR * 2 ;//PB1  set PWM data (1000 - 2000)*2 to (2000 - 4000)
   OCR1B = motor_FrontL * 2 ;//PB2 set PWM data (1000 - 2000)*2 to (2000 - 4000)
   //OCR2A = motor_BackLf / 16 ;//PB3
}
void motor_Mix(){
  if(CH_THR < 1300){
      motor_FrontR = CH_AIL;//Front R
      motor_FrontL = CH_ELE;//Front L
      roll_I_rate = 0.0;
      pitch_I_rate = 0.0;
  }
    if(CH_THR > 1300 && CH_THR < 1700){
      motor_FrontR = CH_AIL + u3_pitch + u2_roll;//Front R
      motor_FrontL = CH_ELE - u3_pitch + u2_roll;//Front L
  }
    if(CH_THR >  1700){
      motor_FrontR = CH_AIL_Cal + u3_pitch + u2_roll;//Front R
      motor_FrontL = CH_ELE_Cal - u3_pitch + u2_roll;//Front L
  }
      motor_FrontR = constrain(motor_FrontR, MINCOMMAND, MAXCOMMAND);
      motor_FrontL = constrain(motor_FrontL, MINCOMMAND, MAXCOMMAND);
}
