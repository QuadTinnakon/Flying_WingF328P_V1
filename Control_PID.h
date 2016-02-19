/*
project_Flying_WingF3_V1  
by: tinnakon kheowree  
tinnakon_za@hotmail.com
tinnakonza@gmail.com
http://quad3d-tin.lnwshop.com
https://www.facebook.com/tinnakonza
*/

float roll_I_rate;
float roll_D_rate;
float err_roll_rate;
float err_roll_ant_rate;

float pitch_I_rate;
float pitch_D_rate;
float err_pitch_rate;
float err_pitch_ant_rate;

//PID mode control
float u2_roll = 0.0;
float u3_pitch = 0.0;

void Control_PIDRate(){
            // ROLL CONTROL
            float Set_roll = SET_Roll1*K_Rate;
            applyDeadband(Set_roll, 7.2);//10 = 2.5
            applyDeadband(GyroXf, 0.21);
            err_roll_rate = (Set_roll - GyroXf);
            roll_I_rate += err_roll_rate*Ki_rateRoll*G_Dt; 
            roll_I_rate = constrain(roll_I_rate, -80, 80);//50 
            roll_D_rate = (tar*roll_D_rate/(tar+G_Dt))+((err_roll_rate - err_roll_ant_rate)/(tar+G_Dt)); 
            err_roll_ant_rate = err_roll_rate;        
            u2_roll = (Kp_rateRoll*err_roll_rate + roll_I_rate + Kd_rateRoll*roll_D_rate)*sensorValue2; 
            //u2_roll = constrain(u2_roll*sensorValue2, -150, 150);//+-150
            // PITCH CONTROL
            float Set_pitch = SET_PITCH1*-K_Rate;
            applyDeadband(Set_pitch, 7.2);//10 = 2.5
            applyDeadband(GyroYf, 0.21);
            err_pitch_rate = (Set_pitch - GyroYf);   
            pitch_I_rate += err_pitch_rate*Ki_ratePitch*G_Dt;  
            pitch_I_rate = constrain(pitch_I_rate, -80, 80);//50
            pitch_D_rate = (tar*pitch_D_rate/(tar+G_Dt))+((err_pitch_rate - err_pitch_ant_rate)/(tar+G_Dt));       
            err_pitch_ant_rate = err_pitch_rate;   
            u3_pitch = (Kp_ratePitch*err_pitch_rate + pitch_I_rate + Kd_ratePitch*pitch_D_rate)*sensorValue1;   
            //u3_pitch = constrain(u3_pitch*sensorValue1, -150, 150);//300 //+-150
}
