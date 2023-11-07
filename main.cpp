/* to-do list
- codes needs changing to use ble interrupts instead of polling
- speed control
- decide on final max value for speed and calculate new PID values.  
*/
#include "mbed.h"
#include "QEI.h"
#include "MMA7660.h"


#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt))) //From the arduino standard library.
#define max_speed_flat 0.15 //0.15       1.6 and 175
#define min_speed_flat 0.45
#define max_speed_slope 0.05 //0.1
#define min_speed_slope 0.45

//Object creation
Serial hm10(PA_11, PA_12); //UART6 TX,RX
MMA7660 MMA(D14,D15);
Serial pc(USBTX, USBRX);

AnalogIn Sensor1(PC_0),Sensor2(PC_1),Sensor3(PB_0),Sensor4(PA_4),Sensor5(PA_1),Sensor6(PA_0);
DigitalOut Sensor1_DB(PA_10),Sensor2_DB(PB_3),Sensor3_DB(PB_5),Sensor4_DB(PB_4),Sensor5_DB(PB_10),Sensor6_DB(PA_8);

PwmOut pwm_l(PC_6);
DigitalOut direction_l(PB_14);
DigitalOut bipolar_l(PB_13);

PwmOut pwm_r(PA_15);
DigitalOut direction_r(PC_14);
DigitalOut bipolar_r(PC_8);

QEI motor_left(PC_2,PC_3,NC,256);
QEI motor_right(PC_5 ,PC_4,NC,256);

DigitalOut enable(PB_2);

//Variable declaration
char c;
float E;
bool start;
bool turnaround_status;
float left_pulses, right_pulses;
float speed_left, speed_right;
float sensor_value1,sensor_value2,sensor_value3,sensor_value4,sensor_value5,sensor_value6;
bool no_linevalue;

int x, y, z;                        //three axis acceleration data
double roll = 0.00, pitch = 0.00;     

//PID controller values
float Kp_flat = 1.6; // (0.25/2) (max speed/max_error)
float Ki_flat = 0;
float Kd_flat = 75; //(20*+P_value)
float lasterror_flat = 0;

float goal = 3.0;

float Kp_slope = 4.5; // (0.25/2) (max speed/max_error)
float Ki_slope = 0;
float Kd_slope = 250; //(20*+P_value)
float lasterror_slope = 0;

//Function declaration
float speed();
void left_motor(float duty_cycle);
void right_motor(float duty_cycle);
float left_PID, right_PID;
float sensor_value();
bool ble();
void PID(float location);
bool turnaround();
float turn_sensor_value();
float RP_calculate();
bool no_line();

//main code here on
int main() {
    //speed_check.attach_us(&speed,5);
    hm10.baud(9600);
    pc.baud(9600);
    //motor_left.reset(); //this resets the encoder count
    //motor_right.reset();
    enable = 0;


    while(1) {
        enable = 0;
        ble();

        while(start){
            PID(sensor_value());
            no_line();
            ble();

        }

        while (turnaround_status){
            turnaround();
            ble();
        }
    }
}



bool turnaround(){
   // pc.printf("\nTurnaround");
    motor_left.reset();
    motor_right.reset();
   left_motor(0.5);
    right_motor(0.5);   
    wait(0.2);
    left_motor(0.35);
    right_motor(0.75);
    while(1){
        if (((motor_right.getPulses()) >= 200) && ((motor_left.getPulses()) >= 200))  {
            while(1){
                if ((turn_sensor_value() < 3.3 && turn_sensor_value() > 2.7))  { //
                    start = 1;
                    turnaround_status = 0;
                    return start,turnaround_status;
                }
        }
        }

    
    }
}

  //Roll & Pitch are the angles which rotate by the axis X and y

float RP_calculate(){
  double x_Buff = float(MMA.x());
  double y_Buff = float(MMA.y());
  double z_Buff = float(MMA.z());
  roll = atan2(y_Buff , z_Buff) * 57.3;
  pitch = atan2((- x_Buff) , sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * 57.3;
  return pitch;
}

void PID(float location){

    if (RP_calculate() >= 10){ //this is where on slope
        //pc.printf("\nSlope");
        float error_slope = goal - location;
        float adjustment_slope = (Kp_slope*error_slope) + (Ki_slope*(Ki_slope + error_slope)) + Kd_slope*(error_slope - lasterror_slope);
        lasterror_slope = error_slope;
        left_PID = constrain(max_speed_slope - adjustment_slope,max_speed_slope,min_speed_slope);
        right_PID = constrain(max_speed_slope + adjustment_slope,max_speed_slope,min_speed_slope);
    }
    else{
        //pc.printf("\nFlat");
        float error_flat = goal - location;
        float adjustment_flat = (Kp_flat*error_flat) + (Ki_flat*(Ki_flat + error_flat)) + Kd_flat*(error_flat - lasterror_flat);
        lasterror_flat = error_flat;
        left_PID = constrain(max_speed_flat - adjustment_flat,max_speed_flat,min_speed_flat);
        right_PID = constrain(max_speed_flat + adjustment_flat,max_speed_flat,min_speed_flat);
    }

    left_motor(left_PID);
    right_motor(right_PID);


}


void left_motor(float duty_cycle){
    enable = 1;
    pwm_l.period_us(50);
    pwm_l.write(duty_cycle);
    //direction_l = 1;
    bipolar_l = 1;
    //printf("\n Left MOTOR \n");
    }

void right_motor(float duty_cycle){
    enable = 1;
    pwm_r.period_us(50);
    pwm_r.write(duty_cycle);
   // direction_r = 0;
    bipolar_r = 1;
    //printf("\nRIGHT MOTOR \n");
    }

float sensor_value(){
    Sensor1_DB.write(1);
    Sensor2_DB.write(1);
    Sensor3_DB.write(1);
    Sensor4_DB.write(1);
    Sensor5_DB.write(1);
    Sensor6_DB.write(1);

    sensor_value1 = Sensor1.read();
    sensor_value2 = Sensor2.read();
    sensor_value3 = Sensor3.read();
    sensor_value4 = Sensor4.read();
    sensor_value5 = Sensor5.read();


    if (sensor_value1 <0.5){sensor_value1 = 0.1;}
    if (sensor_value2 <0.5){sensor_value2 = 0.1;}
    if (sensor_value3 <0.5){sensor_value3 = 0.1;}
    if (sensor_value4 <0.5){sensor_value4 = 0.1;}
    if (sensor_value5 <0.5){sensor_value5 = 0.1;}

   // pc.printf("\n\nS1 = %f",sensor_value1);
    //pc.printf("\nS2 = %f",sensor_value2);
    //pc.printf("\nS3 = %f",sensor_value3);
    //pc.printf("\nS4 = %f",sensor_value4);
    //pc.printf("\nS5 = %f",sensor_value5);


    /* E = ((((sensor_value1)*(1))+((sensor_value5)*(5)))/(sensor_value1 + sensor_value5)); */


    E = ((((sensor_value1)*(1))+((sensor_value2)*(2))+((sensor_value3)*(3))+((sensor_value4)*(4))+((sensor_value5)*(5)))/(sensor_value1 + sensor_value2 + sensor_value3 + sensor_value4 + sensor_value5)); 

    /*E = ((((Sensor1.read())*(1))+((Sensor2.read())*(2))+((Sensor3.read())*(3))+((Sensor4.read())*(4))+((Sensor5.read())*(5)))/(Sensor1.read()+Sensor2.read()+Sensor3.read()+Sensor4.read()+Sensor5.read())); */

   //pc.printf("\n\n\n\n\nSensor1     Sensor2       Sensor3     Sensor4     Sensor5     Sensor6");
    //pc.printf("\n---------------------------------------------------------------------");
    //pc.printf("\n%f    %f      %f    %f    %f    %f",Sensor1.read(),Sensor2.read(),Sensor3.read(),Sensor4.read(),Sensor5.read(),Sensor6.read());

    //pc.printf("\n\nE = %f",E);
    
    //wait(0.5);

    return E;
    }


float turn_sensor_value(){
    Sensor1_DB.write(1);
    Sensor2_DB.write(1);
    Sensor3_DB.write(1);
    Sensor4_DB.write(1);
    Sensor5_DB.write(1);
    Sensor6_DB.write(1);

    sensor_value1 = Sensor1.read();
    sensor_value2 = Sensor2.read();
    sensor_value3 = Sensor3.read();
    sensor_value4 = Sensor4.read();
    sensor_value5 = Sensor5.read();


    if (sensor_value1 <0.5f){sensor_value1 = 0;}
    if (sensor_value2 <0.5f){sensor_value2 = 0;}
    if (sensor_value3 <0.5f){sensor_value3 = 0;}
    if (sensor_value4 <0.5f){sensor_value4 = 0;}
    if (sensor_value5 <0.5f){sensor_value5 = 0;}

    E = ((((sensor_value1)*(1))+((sensor_value2)*(2))+((sensor_value3)*(3))+((sensor_value4)*(4))+((sensor_value5)*(5)))/(sensor_value1 + sensor_value2 + sensor_value3 + sensor_value4 + sensor_value5)); 

    return E;
    }

bool no_line(){
    Sensor1_DB.write(1);
    Sensor2_DB.write(1);
    Sensor3_DB.write(1);
    Sensor4_DB.write(1);
    Sensor5_DB.write(1);
    Sensor6_DB.write(1);

    sensor_value1 = Sensor1.read();
    sensor_value2 = Sensor2.read();
    sensor_value3 = Sensor3.read();
    sensor_value4 = Sensor4.read();
    sensor_value5 = Sensor5.read();

    if ((sensor_value1 <0.5f) && (sensor_value2 <0.5f) && (sensor_value3 <0.5f) && (sensor_value4 <0.5f) && (sensor_value5 <0.5f)){
        no_linevalue = true;
        start = 0;
        
        //start = 0;
        //stop.attach(&controlled_stop, 1.0);
    }
    else{
        no_linevalue = false;
    }
    return start;

}

bool ble(){

    if(hm10.readable()){
        c = hm10.getc(); //read a single character
        //pc.printf("\n%d",c);
        //should be states
        if(c == 'A'){
            start = 1;
            //pc.printf("%d",start);
            return start;
        }
        else if(c == 'B'){
            start = 0;
            //pc.printf("%d",start);
            return start;
        }
        else if (c == 'C'){
            start = 0;
            turnaround_status = 1;
        }
        else{start =  false;};
    }

    }
