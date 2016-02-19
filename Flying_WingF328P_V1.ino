/*
project_Flying_WingF3_V1  
by: tinnakon kheowree  
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com
https://www.facebook.com/tinnakonza

date: 18-02-2559(2016)  Flying_WingF328P_V1 ,,Mode Manual and Gyro stabilizer

support:  Board MWC-S V1.5
• Atmega328p
• MPU6050C Gyro Accelerometer //400kHz nois gyro +-0.05 deg/s , acc +-0.04 m/s^2

Flying_Wing
                  T
                  I
         D10----  I  ---- D9
              \   I   / 

---------Servo---------
FontRight => D9
FontLeft => D10          
----------rx-----------           
Throttle  => D2
Aileron   => D4
Elevator  => D5
*/
#include <Arduino.h>
#include <Wire.h>
#include "configFlying.h"
#include "PPM_328prx.h"
#include "mpu6050.h"
#include "Control_PID.h"
#include "motorX4.h"

void setup()
{
  Serial.begin(57600);//38400
  Serial.print("TK_Flying_WingF3_V1_200Hz");Serial.println("\t");
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  configureReceiver();
  motor_initialize();
  Wire.begin();
  delay(10);
  mpu6050_initialize();
  delay(10); 
  digitalWrite(13, HIGH);
  TWBR = ((F_CPU / 400000L) - 16) / 2;//change the I2C clock rate to 400kHz 
  delay(10);
     for(uint8_t i=0; i<50; i++) 
    {
     mpu6050_Gyro_Values();
     delay(20);
    }
    digitalWrite(13, LOW);
  sensor_Calibrate();//sensor.h
  RC_Calibrate();//"multi_rxPPM2560.h"
  Serial.print("TK_Quad3D_Run_Roop_200Hz");Serial.println("\t");
  sensorPreviousTime = micros();
  previousTime = micros();
}
void loop()
{
  Dt_sensor = micros() - sensorPreviousTime;///////////Roop sensor/////////
  if(Dt_sensor <= 0)
  {
    Dt_sensor = 1001;
  }
    if(Dt_sensor >= 1000 && gyroSamples < 4)////Collect 3 samples = 2760 us  && gyroSamples < 5  && gyroSamples < 5
    {  
        sensorPreviousTime = micros();
        mpu6050_readGyroSum();
    }
   Dt_roop = micros() - previousTime;// 200 Hz task loop (10 ms)  , 5000 = 0.02626 ms
   if(Dt_roop <= 0)
   {
    Dt_roop = 5001; 
   }   
    if (Dt_roop >= 5000) 
    {
      previousTime = micros();
      G_Dt = Dt_roop*0.000001;
      frameCounter++;
      mpu6050_Get_gyro();
////////////////Moving Average Filters///////////////////////////
      GyroXf = (GyroX + GyroX2)/2.0;
      GyroYf = (GyroY + GyroY2)/2.0;
      //GyroZf = (GyroZ + GyroZ2)/2.0;
      GyroX2 = GyroX;GyroY2 = GyroY;//GyroZ2 = GyroZ;//gyro Old1  
      
//PID modeControl///////////
     Control_PIDRate();
//////Out motor///////////
//armed = 1;
     motor_Mix();
/////////////////////////
     motor_command(); 
////////end Out motor//////
 if (frameCounter % TASK_50HZ == 0)// 50 Hz tak (20 ms)
 {
  computeRC();
  SET_Roll1 = (CH_AIL-CH_AIL_Cal) + (CH_ELE-CH_ELE_Cal) + abs(SET_PITCH1)*0.09375;
  SET_PITCH1 = (CH_ELE-CH_ELE_Cal) - (CH_AIL-CH_AIL_Cal) - abs(SET_Roll1)*0.054;
 //end  ARM and DISARM your helicopter 3D///////////////      
}//end roop 50 Hz 
         if (frameCounter % TASK_10HZ == 0)//roop print  ,TASK_noHZ TASK_5HZ  TASK_10HZ
        {
            Serial.print(SET_Roll1);Serial.print("\t");
            Serial.print(SET_PITCH1);Serial.print("\t"); 
            
            Serial.print(CH_THR);Serial.print("\t");
            Serial.print(CH_AIL);Serial.print("\t");  
            Serial.print(CH_ELE);Serial.print("\t");
            //Serial.print(CH_RUD);Serial.print("\t");  
            //Serial.print(AUX_1);Serial.print("\t"); 
            //Serial.print(uAltitude);Serial.print("\t");
            
            //Serial.print(setpoint_rate_roll);Serial.print("\t");
            //Serial.print(setpoint_rate_pitch);Serial.print("\t"); 

            //Serial.print(GyroX*RAD_TO_DEG);Serial.print("\t");
            Serial.print(GyroX);Serial.print("\t");
            //Serial.print(roll_D_rate);Serial.print("\t");
            //Serial.print(GyroY*RAD_TO_DEG);Serial.print("\t");
            Serial.print(GyroY);Serial.print("\t");
            //Serial.print(GyroZf*RAD_TO_DEG);Serial.print("\t");
            //Serial.print(GyrofY);Serial.print("\t");  
            //Serial.print(GyroZ);Serial.print("\t");  
            //Serial.print(gyroRaw[XAXIS]);Serial.print("\t");
            //Serial.print(gyroRaw[YAXIS]);Serial.print("\t");
            //Serial.print(gyroRaw[ZAXIS]);Serial.print("\t");
            
            //Serial.print(err_pitch_rate);Serial.print("\t");
            //Serial.print(roll_I_rate);Serial.print("\t");
            //Serial.print(pitch_I_rate);Serial.print("\t");
            //Serial.print(yaw_I_rate);Serial.print("\t");
            
            Serial.print(motor_FrontR);Serial.print("\t");     
            //Serial.print(motor_FrontLf);Serial.print("\t");
            Serial.print(motor_FrontL);Serial.print("\t");
            
            Serial.print(sensorValue1);Serial.print("\t");
            Serial.print(sensorValue2);Serial.print("\t");
            
            Serial.print(gyroSamples2);Serial.print("\t");
            Serial.print(G_Dt*1000);Serial.print("\t");
            //Serial.print(millis()/1000.0);//millis() micros()
            Serial.print("\n"); 
        }//end roop 5 Hz 
        if (frameCounter >= TASK_1HZ) { // Reset frameCounter back to 0 after reaching 100 (1s)
            frameCounter = 0;
            sensorValue1 = (analogRead(A0))*0.002;
            sensorValue2 = analogRead(A2)*0.002;
              if(Status_LED == LOW)
            {
            Status_LED = HIGH;
            }
            else
            {
            Status_LED = LOW;
            }
            digitalWrite(13, Status_LED);//A
        }//end roop 1 Hz
    }//end roop 100 HZ 
}
